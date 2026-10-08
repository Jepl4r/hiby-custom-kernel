// SPDX-License-Identifier: GPL-2.0
//
// sa_hgl_dma -- /dev/sa_hgl_dma: a block of physically contiguous memory
// from rmem_manager.ko that a program maps, and 2D copies out of it by the
// DMA controller (memory to memory, interleaved).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameter (the stock sa_hgl_dma.sh passes it):
//
//   sahd_hgl_mem_size   bytes of the block, rounded up to a page
//
// One open at a time. mmap() maps the block uncached. A write of a struct
// sahd_copy copies height lines of width bytes from the block at src to the
// physical address dst, the lines stride bytes apart, and returns when the
// DMA has finished.

#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/io.h>

// Exported by rmem_manager.ko.
extern unsigned long rmem_alloc_aligned(int size, int align);
extern void rmem_free(unsigned long addr, int size);

// Lines per DMA transfer when a stride is negative.
#define SAHD_NEG_STRIDE_LINES	128

struct sahd_copy {
	u32 src;		// offset in the block
	u32 dst;		// physical address
	s16 src_stride;
	s16 dst_stride;
	u16 width;
	u16 height;
};

static struct {
	int size;
	unsigned long virt;	// KSEG0
	dma_addr_t phys;
	struct dma_chan *chan;
	struct dma_interleaved_template *xt;
	struct completion done;
} sahd;

static int sahd_hgl_mem_size = -1;
module_param(sahd_hgl_mem_size, int, 0644);

static void sahd_dma_callback(void *param)
{
	complete(param);
}

static int sahd_open(struct inode *inode, struct file *file)
{
	struct dma_interleaved_template *xt;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	unsigned long mem;
	int size, ret;

	if (sahd.chan) {
		printk("[SAUD]sahd_open: hgl dma is opened\n");
		return -EINVAL;
	}
	size = PAGE_ALIGN(sahd_hgl_mem_size);
	mem = rmem_alloc_aligned(size, PAGE_SIZE);
	if (!mem) {
		printk("[SAUD]sahd_open: malloc hgl memory error\n");
		return -ENOMEM;
	}
	xt = kmalloc(sizeof(*xt) + sizeof(struct data_chunk), GFP_KERNEL);
	if (!xt) {
		printk("[SAUD]sahd_open: dma_interleaved_template malloc error\n");
		ret = -ENOMEM;
		goto free_mem;
	}
	dma_cap_zero(mask);
	dma_cap_set(DMA_INTERLEAVE, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		printk("[SAUD]sahd_open: dma_request_channel error\n");
		kfree(xt);
		ret = -EFAULT;
		goto free_mem;
	}

	init_completion(&sahd.done);
	sahd.virt = mem;
	sahd.phys = virt_to_phys((void *)mem);
	sahd.size = size;
	sahd.xt = xt;
	sahd.chan = chan;
	return 0;

free_mem:
	rmem_free(mem, size);
	return ret;
}

static int sahd_release(struct inode *inode, struct file *file)
{
	if (!sahd.chan) {
		printk("[SAUD]sahd_release: hgl dma is closed\n");
		return -EINVAL;
	}
	dma_release_channel(sahd.chan);
	kfree(sahd.xt);
	rmem_free(sahd.virt, sahd.size);
	sahd.chan = NULL;
	sahd.xt = NULL;
	sahd.virt = 0;
	return 0;
}

static int sahd_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off > (unsigned long)sahd.size || len > sahd.size - off) {
		printk("[SAUD]sahd_mmap: len error\n");
		return -EINVAL;
	}
	vma->vm_pgoff = (sahd.phys + off) >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, len, vma->vm_page_prot)) {
		printk("[SAUD]sahd_mmap: io_remap_pfn_range error\n");
		return -EAGAIN;
	}
	return 0;
}

static ssize_t sahd_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct dma_interleaved_template *xt = sahd.xt;
	struct dma_chan *chan = sahd.chan;
	struct dma_async_tx_descriptor *desc;
	struct sahd_copy req;
	int lines;

	if (count != sizeof(req)) {
		printk("[SAUD]sahd_write: len(%d) invalid\n", count);
		return -EINVAL;
	}
	if (copy_from_user(&req, ubuf, sizeof(req))) {
		printk("[SAUD]sahd_write: len(%d) copy_from_user error\n", sizeof(req));
		return -EFAULT;
	}
	if (!req.height || !req.width) {
		printk("[SAUD]sahd_write: params invalid\n");
		return -EFAULT;
	}

	lines = req.height;
	if (lines > SAHD_NEG_STRIDE_LINES && (req.dst_stride < 0 || req.src_stride < 0))
		lines = SAHD_NEG_STRIDE_LINES;

	xt->src_start = sahd.phys + req.src;
	xt->dst_start = req.dst;
	xt->dir = DMA_MEM_TO_MEM;
	xt->src_inc = true;
	xt->dst_inc = true;
	xt->src_sgl = true;
	xt->dst_sgl = true;
	xt->frame_size = 1;
	xt->sgl[0].size = req.width;
	xt->sgl[0].icg = 0;
	xt->sgl[0].dst_icg = req.dst_stride - req.width;
	xt->sgl[0].src_icg = req.src_stride - req.width;

	while (req.height) {
		lines = min_t(int, lines, req.height);
		xt->numf = lines;
		desc = dmaengine_prep_interleaved_dma(chan, xt, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc)
			goto prep_err;
		desc->callback = sahd_dma_callback;
		desc->callback_param = &sahd.done;
		if (dmaengine_submit(desc) < 0)
			goto prep_err;
		dma_async_issue_pending(chan);
		wait_for_completion(&sahd.done);

		req.height -= lines;
		xt->src_start += req.src_stride * lines;
		xt->dst_start += req.dst_stride * lines;
	}
	return sizeof(req);

prep_err:
	printk("[SAUD]sahd_write: dmaengine_prep_interleaved_dma error\n");
	return -EFAULT;
}

static const struct file_operations sahd_fops = {
	.owner = THIS_MODULE,
	.open = sahd_open,
	.release = sahd_release,
	.mmap = sahd_mmap,
	.write = sahd_write,
};

static struct miscdevice sahd_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sa_hgl_dma",
	.fops = &sahd_fops,
};

static int __init sahd_init(void)
{
	int ret;

	printk("[SAUD]sahd_init\n");
	if (sahd_hgl_mem_size <= 0) {
		printk("[SAUD]params error\n");
		return -EINVAL;
	}
	sahd.virt = 0;
	sahd.chan = NULL;
	sahd.xt = NULL;
	ret = misc_register(&sahd_mdev);
	if (ret < 0) {
		printk("[SAUD]misc_register error\n");
		return ret;
	}
	return 0;
}

static void __exit sahd_exit(void)
{
	misc_deregister(&sahd_mdev);
}

module_init(sahd_init);
module_exit(sahd_exit);

MODULE_DESCRIPTION("Smartaction user dma driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
