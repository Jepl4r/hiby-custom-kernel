// SPDX-License-Identifier: GPL-2.0
//
// soc_efuse -- the efuse of the Ingenic X1600: reads its segments for
// /proc/jz/efuse/efuse_chip_id and efuse_user_id, and serves the misc device
// /dev/efuse-string-version.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameter (the stock soc_efuse.sh passes it):
//
//   gpio_efuse_vddq   pin that powers the efuse for programming, active low
//                     (-1: none, and nothing can be written)
//
// The proc files print "NAME: " and the segment in hex. The misc device takes
// these ioctls ('l', nr):
//
//   0xc8  read a segment     struct efuse_wr_info: name, bytes, offset, buf
//   0xc9  write a segment    the same; programs the whole segment
//   0xca  segment size       arg: the name
//   0xcb  segment by number  struct efuse_seg_info: in id, len and a name
//                            buffer of len bytes; out the segment's start
//                            in id and its size in len
//   0xcc  number of segments
//
// Errors return -1, as the vendor driver does.
//
// Loaded after utils.ko, which provides local_clock_ms() and the pin
// parameter.

#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/addrspace.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern u64 local_clock_ms(void);

// Exported by the kernel: a directory under /proc/jz.
extern struct proc_dir_entry *jz_proc_mkdir(char *name);

#define EFUSE_BASE		((void __iomem *)CKSEG1ADDR(0x13540000))
#define EFUSE_CTRL		0x00
#define EFUSE_CFG		0x04
#define EFUSE_STATE		0x08
#define EFUSE_DATA(n)		(0x0c + 4 * (n))

#define EFUSE_CTRL_ADDR_SHIFT	21	// byte address
#define EFUSE_CTRL_LEN_SHIFT	16	// bytes - 1
#define EFUSE_CTRL_PG_EN	BIT(15)
#define EFUSE_CTRL_WR_EN	BIT(1)
#define EFUSE_CTRL_RD_EN	BIT(0)
#define EFUSE_STATE_WR_DONE	BIT(1)
#define EFUSE_STATE_RD_DONE	BIT(0)

#define EFUSE_CHUNK		32	// bytes per read or write
#define EFUSE_MAX_SEGMENT	128
#define EFUSE_WRITE_TIMEOUT_MS	100

#define EFUSE_CMD_READ		_IOWR('l', 0xc8, int)
#define EFUSE_CMD_WRITE		_IOWR('l', 0xc9, int)
#define EFUSE_CMD_SIZE		_IOWR('l', 0xca, int)
#define EFUSE_CMD_INFO		_IOWR('l', 0xcb, int)
#define EFUSE_CMD_COUNT		_IO('l', 0xcc)

struct efuse_segment {
	unsigned int start;	// byte address
	unsigned int size;
	const char *name;
};

static const struct efuse_segment segments[] = {
	{ 0x00, 16, "CHIP_ID" },
	{ 0x10, 11, "CUSTOMER_ID" },
	{ 0x1b, 16, "TRIM_DATA" },
	{ 0x2b,  2, "SOC_INFO" },
	{ 0x2d,  1, "HIDE_BLK" },
	{ 0x2e,  2, "PROGRAM_PROTECT" },
	{ 0x30, 32, "CHIP_KEY" },
	{ 0x50, 32, "USER_KEY" },
	{ 0x70, 16, "NKU" },
};

// A read or write of part of a segment: bytes at offset, to or from buf.
struct efuse_wr_info {
	const char *seg_name;
	int bytes;
	int offset;
	void *buf;
};

struct efuse_seg_info {
	int id;
	int len;
	char __user *name;
};

static int gpio_efuse_vddq = -1;
module_param_cb(gpio_efuse_vddq, &param_gpio_ops, &gpio_efuse_vddq, 0644);

static struct {
	struct miscdevice mdev;
	struct mutex lock;
} efuse;

static struct proc_dir_entry *efuse_dir;
static struct clk *h2clk;
static struct clk *clk;

// The size of the named segment, and its number in *id; -1 if there is none.
static int soc_efuse_read_segment_size(const char *name, int *id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(segments); i++)
		if (!strcmp(segments[i].name, name))
			break;
	if (i == ARRAY_SIZE(segments)) {
		printk(KERN_ERR "EFUSE: can not find %s segment_id\n", name);
		printk(KERN_ERR "EFUSE: can not get %s segment_size\n", name);
		return -1;
	}
	if (id)
		*id = i;
	return segments[i].size;
}

// The segment named in info, checked against info's part of it; -1 if the
// part does not fit.
static int efuse_segment_of(struct efuse_wr_info *info, int *id)
{
	int size = soc_efuse_read_segment_size(info->seg_name, id);

	if (size < 0)
		return -1;
	if (info->offset < 0 || info->bytes < 0 || size < info->offset + info->bytes) {
		printk(KERN_ERR "EFUSE: %d segment_size should be less than %d Byte", *id, size);
		return -1;
	}
	if (segments[*id].size != size) {
		printk(KERN_ERR "EFUSE: %d segment_size should be equal to %d Byte\n", *id, segments[*id].size);
		return -1;
	}
	return size;
}

// Reads info->bytes at info->offset of the segment into info->buf, a kernel
// buffer.
static int soc_efuse_read(struct efuse_wr_info *info)
{
	u8 buf[EFUSE_MAX_SEGMENT];
	unsigned int addr;
	u8 *p = buf;
	int id, size, ret = -1;

	memset(buf, 0, sizeof(buf));
	mutex_lock(&efuse.lock);

	size = efuse_segment_of(info, &id);
	if (size < 0)
		goto out;

	for (addr = segments[id].start; size > 0; size -= EFUSE_CHUNK, addr += EFUSE_CHUNK) {
		int n = min(size, EFUSE_CHUNK);
		u32 ctrl = (n - 1) << EFUSE_CTRL_LEN_SHIFT | addr << EFUSE_CTRL_ADDR_SHIFT;
		int i, k;

		writel(0, EFUSE_BASE + EFUSE_CTRL);
		writel(0, EFUSE_BASE + EFUSE_STATE);
		writel(ctrl, EFUSE_BASE + EFUSE_CTRL);
		writel(ctrl | EFUSE_CTRL_RD_EN, EFUSE_BASE + EFUSE_CTRL);
		while (!(readl(EFUSE_BASE + EFUSE_STATE) & EFUSE_STATE_RD_DONE))
			;

		for (i = 0, k = n; k > 0; i++) {
			int c = min(k, 4);
			u32 word = readl(EFUSE_BASE + EFUSE_DATA(i));

			memcpy(p, &word, c);
			p += c;
			k -= c;
		}
		writel(0, EFUSE_BASE + EFUSE_STATE);
	}

	memcpy(info->buf, buf + info->offset, info->bytes);
	ret = 0;
out:
	mutex_unlock(&efuse.lock);
	return ret;
}

// Writes info->bytes from info->buf, a kernel buffer, at info->offset of the
// segment, and programs the whole segment. Programming only sets bits.
static int soc_efuse_write(struct efuse_wr_info *info)
{
	u8 buf[EFUSE_MAX_SEGMENT];
	unsigned int addr;
	u8 *p = buf;
	int id, size, ret = -1;

	memset(buf, 0, sizeof(buf));
	mutex_lock(&efuse.lock);

	size = soc_efuse_read_segment_size(info->seg_name, &id);
	if (size < 0)
		goto out;
	if (info->offset < 0 || info->bytes < 0 || size < info->offset + info->bytes) {
		printk(KERN_ERR "EFUSE: %d segment_size should be less than %d Byte", id, size);
		goto out;
	}
	memcpy(buf + info->offset, info->buf, info->bytes);
	if (segments[id].size != size) {
		printk(KERN_ERR "EFUSE: %d segment_size should be equal to %d Byte\n", id, segments[id].size);
		goto out;
	}
	if (gpio_efuse_vddq == -1) {
		printk(KERN_ERR "EFUSE: efuse vddq gpio should be set when write segment!\n");
		goto out;
	}

	for (addr = segments[id].start; size > 0; size -= EFUSE_CHUNK, addr += EFUSE_CHUNK) {
		int n = min(size, EFUSE_CHUNK);
		u32 ctrl = (n - 1) << EFUSE_CTRL_LEN_SHIFT | addr << EFUSE_CTRL_ADDR_SHIFT | EFUSE_CTRL_PG_EN;
		u64 start;
		int i, k;

		for (i = 0, k = n; k > 0; i++) {
			int c = min(k, 4);
			u32 word = 0;

			memcpy(&word, p, c);
			writel(word, EFUSE_BASE + EFUSE_DATA(i));
			p += c;
			k -= c;
		}

		writel(0, EFUSE_BASE + EFUSE_CTRL);
		writel(0, EFUSE_BASE + EFUSE_STATE);
		writel(ctrl, EFUSE_BASE + EFUSE_CTRL);

		start = local_clock_ms();
		gpio_set_value(gpio_efuse_vddq, 0);
		writel(readl(EFUSE_BASE + EFUSE_CTRL) | EFUSE_CTRL_WR_EN, EFUSE_BASE + EFUSE_CTRL);
		while (!(readl(EFUSE_BASE + EFUSE_STATE) & EFUSE_STATE_WR_DONE)) {
			if (local_clock_ms() - start > EFUSE_WRITE_TIMEOUT_MS) {
				printk(KERN_ERR "EFUSE: write efuse timeout\n");
				break;
			}
		}
		gpio_set_value(gpio_efuse_vddq, 1);
	}
	ret = 0;
out:
	mutex_unlock(&efuse.lock);
	return ret;
}

static int efuse_read_segment_proc(struct seq_file *m, const char *name)
{
	u8 buf[EFUSE_MAX_SEGMENT];
	struct efuse_wr_info info;
	int size, ret, i;

	memset(buf, 0, sizeof(buf));
	size = soc_efuse_read_segment_size(name, NULL);
	if (size < 0)
		return -1;

	info.seg_name = name;
	info.bytes = size;
	info.offset = 0;
	info.buf = buf;
	ret = soc_efuse_read(&info);
	if (ret < 0) {
		printk(KERN_ERR "EFUSE: failed to read %s segment\n", name);
		return ret;
	}

	seq_printf(m, "%s: ", name);
	for (i = 0; i < size; i++)
		seq_printf(m, "%02x", buf[i]);
	seq_printf(m, "\n");
	return 0;
}

static int efuse_read_user_id_proc(struct seq_file *m, void *v)
{
	return efuse_read_segment_proc(m, "CUSTOMER_ID");
}

static int efuse_read_chip_id_proc(struct seq_file *m, void *v)
{
	return efuse_read_segment_proc(m, "CHIP_ID");
}

static int efuse_read_userID_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, efuse_read_user_id_proc, PDE_DATA(inode));
}

static int efuse_read_chipID_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, efuse_read_chip_id_proc, PDE_DATA(inode));
}

static const struct file_operations efuse_proc_read_userID_fops = {
	.owner = THIS_MODULE,
	.llseek = seq_lseek,
	.read = seq_read,
	.open = efuse_read_userID_proc_open,
	.release = single_release,
};

static const struct file_operations efuse_proc_read_chipID_fops = {
	.owner = THIS_MODULE,
	.llseek = seq_lseek,
	.read = seq_read,
	.open = efuse_read_chipID_proc_open,
	.release = single_release,
};

// A segment name from user space; one too long names no segment.
static int efuse_user_name(char *name, size_t size, const char __user *uname)
{
	long n = strncpy_from_user(name, uname, size);

	if (n < 0)
		return -1;
	name[size - 1] = '\0';
	return 0;
}

static long efuse_user_rw(unsigned int cmd, void __user *arg)
{
	struct efuse_wr_info info;
	char name[32];
	u8 buf[EFUSE_MAX_SEGMENT];
	void __user *ubuf;
	int ret;

	if (copy_from_user(&info, arg, sizeof(info)))
		return -1;
	if (efuse_user_name(name, sizeof(name), (const char __user *)info.seg_name))
		return -1;
	if (info.bytes < 0 || info.bytes > sizeof(buf))
		info.bytes = sizeof(buf) + 1;	// fails the segment size check
	ubuf = (void __user *)info.buf;
	info.seg_name = name;
	info.buf = buf;

	if (cmd == EFUSE_CMD_WRITE) {
		if (info.bytes <= sizeof(buf) && copy_from_user(buf, ubuf, info.bytes))
			return -1;
		return soc_efuse_write(&info);
	}

	ret = soc_efuse_read(&info);
	if (!ret && copy_to_user(ubuf, buf, info.bytes))
		return -1;
	return ret;
}

static long efuse_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct efuse_seg_info si;
	char name[32];
	int n;

	switch (cmd) {
	case EFUSE_CMD_READ:
	case EFUSE_CMD_WRITE:
		return efuse_user_rw(cmd, uarg);
	case EFUSE_CMD_SIZE:
		if (efuse_user_name(name, sizeof(name), uarg))
			return -1;
		return soc_efuse_read_segment_size(name, NULL);
	case EFUSE_CMD_INFO:
		if (copy_from_user(&si, uarg, sizeof(si)))
			return -1;
		if (si.id < 0 || si.id >= ARRAY_SIZE(segments))
			return -1;
		mutex_lock(&efuse.lock);
		n = min_t(int, si.len, strlen(segments[si.id].name) + 1);
		if (n > 0 && copy_to_user(si.name, segments[si.id].name, n))
			n = -1;
		si.len = segments[si.id].size;
		si.id = segments[si.id].start;
		mutex_unlock(&efuse.lock);
		if (n < 0 || copy_to_user(uarg, &si, sizeof(si)))
			return -1;
		return 0;
	case EFUSE_CMD_COUNT:
		return ARRAY_SIZE(segments);
	default:
		printk(KERN_ERR "EFUSE: do not support this cmd: %x\n", cmd);
		return -1;
	}
}

static int efuse_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int efuse_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations efuse_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = efuse_ioctl,
	.open = efuse_open,
	.release = efuse_release,
};

// The read and program timings, in cycles of the AHB2 clock.
static int efuse_timing(unsigned int ns, u32 *cfg)
{
	unsigned int rd_adj;
	int rd_strobe, wr_strobe;

	rd_adj = 7 / ns;
	if ((rd_adj + 1) * ns < 8) {
		printk(KERN_ERR "EFUSE: failed to get efuse cfg rd_adj!\n");
		return -1;
	}

	rd_strobe = (int)(35 / ns) - 4 - rd_adj;
	if (rd_strobe < 0)
		rd_strobe = 0;
	if ((rd_adj + 5 + rd_strobe) * ns < 36) {
		printk(KERN_ERR "EFUSE: failed to get efuse cfg rd_strobe!\n");
		return -1;
	}

	// The program pulse: 9 to 11 us.
	wr_strobe = (int)(10000 / ns) - 1666 - rd_adj;
	if (wr_strobe < 0)
		wr_strobe = 0;
	wr_strobe++;
	if ((rd_adj + 1666 + wr_strobe) * ns - 9000 > 2000) {
		printk(KERN_ERR "EFUSE: failed to get efuse cfg wr_strobe!\n");
		return -1;
	}

	*cfg = rd_adj << 20 | rd_strobe << 16 | rd_adj << 12 | wr_strobe;
	return 0;
}

static int __init jz_efuse_init(void)
{
	unsigned int ns;
	u32 cfg;
	int ret;

	h2clk = clk_get(NULL, "div_ahb2");
	if (IS_ERR(h2clk)) {
		printk(KERN_ERR "EFUSE: failed to get h2clk!\n");
		return -1;
	}
	ns = 1000000000 / clk_get_rate(h2clk);

	// VDDQ stays off except while programming.
	if (gpio_efuse_vddq != -1) {
		gpio_request(gpio_efuse_vddq, "efuse_vddq");
		gpio_direction_output(gpio_efuse_vddq, 1);
	}
	mutex_init(&efuse.lock);

	ret = efuse_timing(ns, &cfg);
	if (ret)
		goto err_h2clk;

	clk = clk_get(NULL, "gate_efuse");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "EFUSE: failed to get efuse clk!\n");
		ret = -1;
		goto err_h2clk;
	}
	clk_prepare_enable(clk);
	writel(cfg, EFUSE_BASE + EFUSE_CFG);

	efuse_dir = jz_proc_mkdir("efuse");
	if (!efuse_dir) {
		printk(KERN_ERR "EFUSE: failed to create_proc_entry for common efuse.\n");
		ret = -ENODEV;
		goto err_clk;
	}
	if (!proc_create_data("efuse_chip_id", 0444, efuse_dir, &efuse_proc_read_chipID_fops, NULL))
		printk(KERN_ERR "EFUSE: create proc of efuse_chip_id error!!!\n");
	if (!proc_create_data("efuse_user_id", 0444, efuse_dir, &efuse_proc_read_userID_fops, NULL))
		printk(KERN_ERR "EFUSE: create proc of efuse_user_id error!!!\n");

	efuse.mdev.minor = MISC_DYNAMIC_MINOR;
	efuse.mdev.name = "efuse-string-version";
	efuse.mdev.fops = &efuse_misc_fops;
	ret = misc_register(&efuse.mdev);
	if (ret < 0)
		goto err_proc;
	return 0;

err_proc:
	proc_remove(efuse_dir);
err_clk:
	clk_disable_unprepare(clk);
	clk_put(clk);
err_h2clk:
	if (gpio_efuse_vddq != -1)
		gpio_free(gpio_efuse_vddq);
	clk_put(h2clk);
	return ret;
}
module_init(jz_efuse_init);

static void __exit jz_efuse_exit(void)
{
	misc_deregister(&efuse.mdev);
	proc_remove(efuse_dir);
	clk_disable(clk);
	clk_unprepare(clk);
	clk_put(clk);
	if (gpio_efuse_vddq != -1)
		gpio_free(gpio_efuse_vddq);
	clk_put(h2clk);
}
module_exit(jz_efuse_exit);

MODULE_DESCRIPTION("Efuse of the Ingenic X1600");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
