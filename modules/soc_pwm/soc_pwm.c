// SPDX-License-Identifier: GPL-2.0
//
// soc_pwm -- the eight channels of the X1600 PWM controller for the other
// HiBy modules (backlight, R1 LEDs), and /dev/jz_pwm.
//
// A drop-in replacement for the vendor module of the same name; the stock
// soc_pwm.sh loads it with no parameters.
//
// A channel is requested by pin (pwm2_request() returns its index), set up
// with pwm2_config() and driven with pwm2_set_level(): level 0 to levels,
// where 0 and levels hold the pin at a constant level through the GPIO
// function. A channel can instead play a buffer of waveform words by DMA
// (pwm2_dma_init(), pwm2_dma_update()), once or in a loop.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/addrspace.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern int gpio_set_func(int gpio, int func);
extern char *gpio_to_str(int gpio, char *buf);
extern int str_to_gpio(const char *str);
extern void __assert(const char *expr, const char *file, int line, const char *func);

// --- registers --------------------------------------------------------------

#define PWM_PHYS		0x134c0000
#define PWM_REG(off)		((void __iomem *)CKSEG1ADDR(PWM_PHYS + (off)))
#define PWM_ENS			0x00		// write 1: channel on
#define PWM_ENC			0x04		// write 1: channel off
#define PWM_UPT			0x10		// write 1: load the waveform word
#define PWM_BUSY		0x14		// 1 while the load is pending
#define PWM_MODE		0x20		// 0 waveform word, 1 DMA
#define PWM_INL			0x24		// output level at the start of a period
#define PWM_IDL			0x28		// output level while idle
#define PWM_CCFG(n)		(0x40 + 4 * (n))	// clock divider, 2^value
#define PWM_WCFG(n)		(0x80 + 4 * (n))	// low count 15:0, high count 31:16
#define PWM_DFIFO(n)		(0xc0 + 4 * (n))	// DMA data
#define PWM_DTRIG(n)		(0x140 + 4 * (n))	// DMA request threshold
#define PWM_DCR			0x180		// DMA requests on

#define PWM_CHANNELS		8
#define PWM_MAX_COUNT		0xffff
#define PWM_DIV_TRIES		8
#define PWM_DMA_THRESHOLD	32
// Request type of PWM channel 0 on the X1600 DMA controller.
#define PWM_DMA_TYPE		44
// Waiting for a waveform load: 1.5 periods' worth of the waveform frequency.
#define PWM_UPDATE_WAIT_US	1500000

enum pwm_mode {
	PWM_MODE_NONE,
	PWM_MODE_LEVEL,
	PWM_MODE_DMA,
};

// The channel configuration of pwm2_config(), as the HiBy modules pass it.
struct pwm2_config {
	int reserved;			// not read
	int idle_level;			// output level while the channel is idle
	int exact_levels;		// nonzero: the period is a whole number of levels
	unsigned long freq;		// Hz
	unsigned long levels;		// the range of pwm2_set_level(), below 0xffff
};

// pwm2_dma_init(): output levels while idle and at the start of a period.
struct pwm2_dma_config {
	int id;
	int idle_level;
	int init_level;
};

// pwm2_dma_update(): count waveform words at buf, played once or in a loop.
// Played once, they are copied into coherent memory and the call returns
// when the DMA has finished; in a loop, buf itself (KSEG0) is played until
// pwm2_dma_disable_loop().
struct pwm2_dma_data {
	u32 *buf;
	int count;
	int loop;
};

// A pin that can carry a channel, and the function that connects it.
struct pwm_gpio {
	const char *name;
	int id;
	int gpio;
	int func;
};

struct pwm_channel {
	const char *label;
	int mode;		// enum pwm_mode
	int requested;
	int enabled;
	int gpio_forced;	// the pin is held as a GPIO output
	int keep_running;	// a constant level leaves the channel on
	int no_enable;		// the channel is never switched on
	unsigned long clk_rate;	// counter clock after the divider
	unsigned long freq;	// clk_rate / period
	u32 duty;
	u32 period;		// counts per waveform period
	u32 levels;
	int idle_level;
	u32 high;		// counts at the start level
	u32 low;		// counts at the other level
	struct pwm_gpio *pin;
	int dma_init_level;
	int dma_loop;
	void *dma_buf;
	u32 dma_size;
	dma_addr_t dma_addr;
	struct mutex lock;
	wait_queue_head_t dma_wait;
	int dma_done;
	struct dma_chan *dma_chan;
};

static struct pwm_channel pwmdata[PWM_CHANNELS];

static struct pwm_gpio pwm_gpio_array[] = {
	{ "pwm6", 6, GPIO_PB(9),  GPIO_FUNC_1 },
	{ "pwm7", 7, GPIO_PB(10), GPIO_FUNC_1 },
	{ "pwm5", 5, GPIO_PB(19), GPIO_FUNC_2 },
	{ "pwm6", 6, GPIO_PB(20), GPIO_FUNC_2 },
	{ "pwm7", 7, GPIO_PB(21), GPIO_FUNC_2 },
	{ "pwm0", 0, GPIO_PC(0),  GPIO_FUNC_0 },
	{ "pwm1", 1, GPIO_PC(1),  GPIO_FUNC_0 },
	{ "pwm2", 2, GPIO_PC(2),  GPIO_FUNC_0 },
	{ "pwm3", 3, GPIO_PC(24), GPIO_FUNC_1 },
	{ "pwm4", 4, GPIO_PC(25), GPIO_FUNC_1 },
	{ "pwm5", 5, GPIO_PC(26), GPIO_FUNC_1 },
};

static unsigned long clk_src_rate;
static struct clk *pwm_mux_clk;
static struct clk *pwm_div_clk;
static struct clk *pwm_gate_clk;
static DEFINE_SPINLOCK(pwm_reg_lock);

static inline u32 pwm_read(unsigned int off)
{
	return readl(PWM_REG(off));
}

static inline void pwm_write(unsigned int off, u32 v)
{
	writel(v, PWM_REG(off));
}

// Sets or clears the channel's bit in one of the per-channel bit registers.
static void pwm_set_bit(unsigned int off, int id, int on)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&pwm_reg_lock, flags);
	v = pwm_read(off);
	v ^= ((u32)!!on << id ^ v) & BIT(id);
	pwm_write(off, v);
	spin_unlock_irqrestore(&pwm_reg_lock, flags);
}

static void set_pwm_mode(int id, int dma)
{
	pwm_set_bit(PWM_MODE, id, dma);
}

static void set_pwm_init_level(int id, int level)
{
	pwm_set_bit(PWM_INL, id, level);
}

static void set_pwm_idle_level(int id, int level)
{
	pwm_set_bit(PWM_IDL, id, level);
}

static void set_pwm_dma_req(int id, int on)
{
	pwm_set_bit(PWM_DCR, id, on);
}

// The divider that brings the source clock to clk_rate; an unsupported ratio
// gets the largest one.
static void pwm_set_clk_div(int id)
{
	u32 div;

	switch (clk_src_rate / pwmdata[id].clk_rate) {
	case 1:
		div = 0;
		break;
	case 2:
		div = 1;
		break;
	case 4:
		div = 2;
		break;
	case 8:
		div = 3;
		break;
	case 16:
		div = 4;
		break;
	case 32:
		div = 5;
		break;
	case 64:
		div = 6;
		break;
	case 128:
		div = 7;
		break;
	default:
		printk(KERN_ERR "PWM: clk div err, pwm ch %d !\n", id);
		div = 7;
		break;
	}
	pwm_write(PWM_CCFG(id), div);
}

static void pwm_enable(int id)
{
	struct pwm_channel *p = &pwmdata[id];

	if (p->enabled)
		return;
	gpio_set_func(p->pin->gpio, p->pin->func);
	if (!p->no_enable)
		pwm_write(PWM_ENS, BIT(id));
	p->enabled = 1;
}

// The DMA channel's current address, if it is inside the buffer; 0 after two
// readings outside it.
dma_addr_t pwm2_get_current_dma_addr(int id)
{
	struct pwm_channel *p = &pwmdata[id];
	dma_addr_t start = p->dma_addr, end = start + p->dma_size;
	struct dma_chan *chan = p->dma_chan;
	dma_addr_t addr;
	int tries;

	for (tries = 2; tries; tries--) {
		addr = chan->device->get_current_trans_addr(chan, NULL, NULL, DMA_MEM_TO_DEV);
		if (addr >= start && addr < end)
			return addr;
	}
	return 0;
}
EXPORT_SYMBOL(pwm2_get_current_dma_addr);

// The period in counts of the clock at rate, halving the clock until the
// period fits in 16 bits, at most seven times. With exact_levels the period
// is rounded down to a whole number of levels.
int pwm2_config(int id, struct pwm2_config *cfg)
{
	unsigned long freq = cfg->freq, levels = cfg->levels;
	int idle = cfg->idle_level;
	struct pwm_channel *p;
	unsigned long rate, period;
	int tries, ret;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	if (!freq) {
		printk(KERN_ERR "PWM: pwm ch %d only support frequency high than 0 Hz\n", id);
		return -1;
	}
	if (clk_src_rate < freq) {
		printk(KERN_ERR "PWM: pwm only support frequency low than %lu\n", clk_src_rate);
		return -1;
	}
	if (levels >= PWM_MAX_COUNT) {
		printk(KERN_ERR "PWM: pwm ch %d levels need less than %d\n", id, PWM_MAX_COUNT);
		return -1;
	}

	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (!p->requested) {
		printk(KERN_ERR "PWM: pwm ch %d is not request!\n", id);
		ret = -1;
		goto out;
	}
	if (p->enabled) {
		printk(KERN_ERR "PWM: pwm ch %d Cannot configure at working\n", id);
		ret = -1;
		goto out;
	}
	p->levels = levels;
	p->idle_level = !!idle;

	rate = clk_src_rate;
	for (tries = PWM_DIV_TRIES;;) {
		period = rate / freq;
		if (period <= PWM_MAX_COUNT || !--tries)
			break;
		rate >>= 1;
	}
	p->clk_rate = rate;
	if (!cfg->exact_levels)
		p->period = period;
	else if (period < levels)
		p->period = levels;
	else
		p->period = period - period % levels;
	if (p->period > PWM_MAX_COUNT)
		p->period = PWM_MAX_COUNT;
	p->freq = rate / p->period;

	pwm_set_clk_div(id);
	set_pwm_idle_level(id, p->idle_level);
	set_pwm_init_level(id, !p->idle_level);
	set_pwm_mode(id, 0);
	p->mode = PWM_MODE_LEVEL;
	ret = 0;
out:
	mutex_unlock(&p->lock);
	return ret;
}
EXPORT_SYMBOL(pwm2_config);

// Stops a looping DMA; the caller holds the channel's lock.
static void pwm_dma_stop_loop(int id)
{
	struct pwm_channel *p = &pwmdata[id];

	set_pwm_dma_req(id, 0);
	dmaengine_terminate_all(p->dma_chan);
	p->dma_buf = NULL;
	p->dma_loop = 0;
}

int pwm2_dma_disable_loop(int id)
{
	struct pwm_channel *p;
	int ret;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (p->mode != PWM_MODE_DMA) {
		printk(KERN_ERR "PWM: pwm ch%d is not config dma mode!\n", id);
		ret = -1;
	} else if (!p->enabled) {
		printk(KERN_ERR "PWM: pwm ch%d dma is not enable\n", id);
		ret = -1;
	} else if (!p->dma_loop) {
		printk(KERN_ERR "PWM: pwm ch%d dma is not loop\n", id);
		ret = -1;
	} else {
		pwm_dma_stop_loop(id);
		ret = 0;
	}
	mutex_unlock(&p->lock);
	return ret;
}
EXPORT_SYMBOL(pwm2_dma_disable_loop);

// Puts the channel in DMA mode on a channel of its own; returns the counter
// clock.
int pwm2_dma_init(int id, struct pwm2_dma_config *cfg)
{
	struct dma_slave_config conf = { 0 };
	struct pwm_channel *p;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int ret;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (!p->requested) {
		printk(KERN_ERR "PWM: pwm ch %d is not request!\n", id);
		ret = -1;
		goto out;
	}
	if (p->enabled) {
		printk(KERN_ERR "PWM: pwm ch %d Cannot configure at working\n", id);
		ret = -1;
		goto out;
	}
	if (p->dma_chan) {
		printk(KERN_ERR "PWM: pwm ch %d is busy\n", id);
		ret = -1;
		goto out;
	}

	p->clk_rate = clk_src_rate;
	pwm_set_clk_div(id);
	p->dma_init_level = !!cfg->init_level;
	p->idle_level = !!cfg->idle_level;
	set_pwm_init_level(id, p->dma_init_level);
	set_pwm_idle_level(id, p->idle_level);
	set_pwm_mode(id, 1);
	pwm_write(PWM_DTRIG(id), PWM_DMA_THRESHOLD);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		printk(KERN_ERR "PWM%d request dma tx channel failed", id);
		ret = -1;
		goto out;
	}
	conf.direction = DMA_MEM_TO_DEV;
	conf.dst_addr = PWM_PHYS + PWM_DFIFO(id);
	conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	conf.src_maxburst = 4;
	conf.dst_maxburst = 4;
	conf.slave_id = PWM_DMA_TYPE + id;
	dmaengine_slave_config(chan, &conf);
	p->dma_chan = chan;
	p->mode = PWM_MODE_DMA;
	ret = p->clk_rate;
out:
	mutex_unlock(&p->lock);
	return ret;
}
EXPORT_SYMBOL(pwm2_dma_init);

static void dma_tx_callback(void *param)
{
	struct pwm_channel *p = &pwmdata[(int)(long)param];

	if (!p->dma_loop) {
		p->dma_done = 1;
		wake_up(&p->dma_wait);
	}
}

// The level the channel is outputting, scaled back to 0..levels.
u32 soc_pwm_get_level(int id)
{
	struct pwm_channel *p;
	u32 count, v;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: %s, No support pwm ch %d !\n", __func__, id);
		return -1;
	}
	p = &pwmdata[id];
	count = p->idle_level ? p->low : p->high;
	if (count && p->period != count && p->enabled) {
		v = pwm_read(PWM_WCFG(id));
		count = p->idle_level ? v & 0xffff : v >> 16;
	}
	return count * p->levels / p->period;
}
EXPORT_SYMBOL(soc_pwm_get_level);

static void pwm_dma_free(struct pwm_channel *p)
{
	dma_free_coherent(NULL, p->dma_size, p->dma_buf, p->dma_addr);
}

int pwm2_dma_update(int id, struct pwm2_dma_data *data)
{
	struct dma_async_tx_descriptor *desc;
	struct scatterlist sg;
	struct pwm_channel *p;
	int ret;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	if (!data || !data->buf || !data->count) {
		printk(KERN_ERR "PWM: pwm ch%d not have dma data!\n", id);
		return -1;
	}
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (p->mode != PWM_MODE_DMA) {
		printk(KERN_ERR "PWM: pwm ch%d is not config dma mode!\n", id);
		ret = -1;
		goto out;
	}
	if (p->dma_loop) {
		printk(KERN_ERR "PWM: pwm ch%d work in dma loop mode\n", id);
		ret = -1;
		goto out;
	}
	if (p->dma_buf)
		__assert("!pwmdata[id].dma_data", __FILE__, __LINE__, __func__);

	p->dma_size = data->count * 4;
	if (!data->loop) {
		p->dma_buf = dma_alloc_coherent(NULL, p->dma_size, &p->dma_addr, GFP_KERNEL);
		if (!p->dma_buf) {
			printk(KERN_ERR "PWM: pwm ch%d dma_alloc_coherent fail\n", id);
			ret = -1;
			goto out;
		}
		memcpy(p->dma_buf, data->buf, p->dma_size);
	} else {
		p->dma_buf = data->buf;
		p->dma_addr = virt_to_phys(data->buf);
	}

	if (data->loop) {
		desc = dmaengine_prep_dma_cyclic(p->dma_chan, p->dma_addr, p->dma_size, p->dma_size,
						 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	} else {
		sg_init_table(&sg, 1);
		sg_dma_address(&sg) = p->dma_addr;
		sg_dma_len(&sg) = p->dma_size;
		desc = dmaengine_prep_slave_sg(p->dma_chan, &sg, 1, DMA_MEM_TO_DEV,
					       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	}
	if (!desc) {
		printk(KERN_ERR "PWM request dma desc failed");
		if (!data->loop)
			pwm_dma_free(p);
		p->dma_buf = NULL;
		ret = -1;
		goto out;
	}

	p->dma_loop = !!data->loop;
	desc->callback = dma_tx_callback;
	desc->callback_param = (void *)(long)id;
	dmaengine_submit(desc);
	p->dma_done = 0;
	set_pwm_dma_req(id, 1);
	pwm_enable(id);
	dma_async_issue_pending(p->dma_chan);
	if (data->loop) {
		ret = 0;
		goto out;
	}

	wait_event(p->dma_wait, p->dma_done);
	set_pwm_dma_req(id, 0);
	dmaengine_terminate_all(p->dma_chan);
	pwm_dma_free(p);
	p->dma_buf = NULL;
	ret = 0;
out:
	mutex_unlock(&p->lock);
	return ret;
}
EXPORT_SYMBOL(pwm2_dma_update);

// The channel on the pin; requesting a channel already held returns it.
int pwm2_request(int gpio, const char *label)
{
	struct pwm_channel *p;
	struct pwm_gpio *pin = NULL;
	char buf[12];
	int i, id;

	for (i = 0; i < ARRAY_SIZE(pwm_gpio_array); i++) {
		if (pwm_gpio_array[i].gpio == gpio) {
			pin = &pwm_gpio_array[i];
			break;
		}
	}
	if (!pin) {
		printk(KERN_ERR "PWM: %s not support as pwm.\n", gpio_to_str(gpio, buf));
		return -1;
	}

	id = pin->id;
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (!p->requested) {
		if (gpio_request(gpio, label)) {
			printk(KERN_ERR "PWM: pwm ch %d, request IO %s error !\n", id,
			       gpio_to_str(gpio, buf));
			mutex_unlock(&p->lock);
			return -1;
		}
		p->label = label;
		p->pin = pin;
		p->clk_rate = clk_src_rate;
		p->requested = 1;
	}
	mutex_unlock(&p->lock);
	return id;
}
EXPORT_SYMBOL(pwm2_request);

int pwm2_release(int id)
{
	struct pwm_channel *p;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (p->mode == PWM_MODE_DMA && p->dma_loop && p->enabled)
		pwm_dma_stop_loop(id);
	if (p->dma_chan) {
		dma_release_channel(p->dma_chan);
		p->dma_chan = NULL;
	}
	if (p->enabled) {
		pwm_write(PWM_ENC, BIT(id));
		p->enabled = 0;
	}
	if (p->requested) {
		gpio_free(p->pin->gpio);
		p->requested = 0;
	}
	p->mode = PWM_MODE_NONE;
	mutex_unlock(&p->lock);
	return 0;
}
EXPORT_SYMBOL(pwm2_release);

// Level 0 to levels, scaled to the period; a level that scales to 0 counts
// is 1 count. A level with no low or no high counts holds the pin as a GPIO
// output and switches the channel off, unless keep_running is set.
int pwm2_set_level(int id, unsigned int level)
{
	struct pwm_channel *p;
	u32 data = 0;
	int constant, ret;

	if ((unsigned int)id >= PWM_CHANNELS) {
		printk(KERN_ERR "PWM: No support pwm ch %d !\n", id);
		return -1;
	}
	p = &pwmdata[id];
	mutex_lock(&p->lock);
	if (p->mode != PWM_MODE_LEVEL) {
		printk(KERN_ERR "PWM: pwm ch%d is not config!\n", id);
		ret = -1;
		goto out;
	}
	if (level > p->levels) {
		printk(KERN_ERR "PWM: pwm ch%d set level more than max level!\n", id);
		ret = -1;
		goto out;
	}
	if (level) {
		level = level * p->period / p->levels;
		if (!level)
			level = 1;
	}
	p->duty = level;
	if (p->idle_level) {
		p->high = p->period - level;
		p->low = level;
	} else {
		p->low = p->period - level;
		p->high = level;
	}

	if (!p->low) {
		p->gpio_forced = 1;
		gpio_set_func(p->pin->gpio, GPIO_OUTPUT1);
		data = (p->period - 1) << 16 | 1;
		constant = 1;
	} else if (!p->high) {
		p->gpio_forced = 1;
		gpio_set_func(p->pin->gpio, GPIO_OUTPUT0);
		data = 1 << 16 | ((p->period - 1) & 0xffff);
		constant = 1;
	} else {
		if (p->gpio_forced) {
			p->gpio_forced = 0;
			gpio_set_func(p->pin->gpio, p->pin->func);
		}
		data = (p->low & 0xffff) | p->high << 16;
		constant = 0;
	}
	pwm_write(PWM_WCFG(id), data);
	pwm_write(PWM_UPT, BIT(id));
	pwm_enable(id);

	if (!p->no_enable) {
		unsigned long us = PWM_UPDATE_WAIT_US / p->freq;

		if (!us)
			us = 1;
		usleep_range(us, us);
		if ((pwm_read(PWM_BUSY) & BIT(id)) >> id) {
			printk(KERN_ERR "PWM: pwm updata wavwfrom config timeout\n");
			ret = -1;
			goto out;
		}
	}
	if (constant && !p->keep_running && p->enabled) {
		pwm_write(PWM_ENC, BIT(id));
		p->enabled = 0;
	}
	ret = 0;
out:
	mutex_unlock(&p->lock);
	return ret;
}
EXPORT_SYMBOL(pwm2_set_level);

// --- /dev/jz_pwm ------------------------------------------------------------

#define PWM_IOC_REQUEST		_IOW('P', 0x0b, char)
#define PWM_IOC_RELEASE		_IOW('P', 0x16, int)
#define PWM_IOC_DMA_DISABLE	_IOW('P', 0x4d, char)
#define PWM_IOC_NO_ENABLE	_IOW('P', 0x58, int)
#define PWM_IOC_KEEP_RUNNING	_IOW('P', 0x59, int)
#define PWM_IOC_ENABLE		_IOW('P', 0x62, int)
#define PWM_IOC_DISABLE		_IOW('P', 0x63, int)
#define PWM_IOC_DMA_INIT	_IOW('P', 0x37, struct pwm2_dma_config)
#define PWM_IOC_CONFIG		_IOW('P', 0x21, struct pwm_ioc_config)
#define PWM_IOC_SET_LEVEL	_IOWR('P', 0x2c, int)
#define PWM_IOC_DMA_UPDATE	_IOWR('P', 0x42, struct pwm_ioc_dma)

struct pwm_ioc_config {
	struct pwm2_config cfg;
	int id;
};

struct pwm_ioc_dma {
	u32 *buf;		// user pointer
	int count;
	int loop;
	int id;
};

static long pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int i, gpio, ret;

	switch (cmd) {
	case PWM_IOC_NO_ENABLE:
	case PWM_IOC_KEEP_RUNNING: {
		int id;

		if (copy_from_user(&id, argp, sizeof(id)))
			goto copy_err;
		if ((unsigned int)id >= PWM_CHANNELS)
			return -1;
		if (cmd == PWM_IOC_NO_ENABLE)
			pwmdata[id].no_enable = 1;
		else
			pwmdata[id].keep_running = 1;
		return 0;
	}
	case PWM_IOC_ENABLE:
	case PWM_IOC_DISABLE: {
		u32 mask = 0;

		if (copy_from_user(&mask, argp, sizeof(mask)))
			goto copy_err;
		pwm_write(cmd == PWM_IOC_ENABLE ? PWM_ENS : PWM_ENC, mask);
		return 0;
	}
	case PWM_IOC_REQUEST: {
		char name[12] = { 0 };

		if (copy_from_user(name, argp, sizeof(name)))
			goto copy_err;
		gpio = str_to_gpio(name);
		if (gpio < 0) {
			printk(KERN_ERR "PWM: gpio is invalid:%s !\n", name);
			return -1;
		}
		for (i = 0; i < ARRAY_SIZE(pwm_gpio_array); i++)
			if (pwm_gpio_array[i].gpio == gpio)
				return pwm2_request(gpio, pwm_gpio_array[i].name);
		printk(KERN_ERR "PWM: gpio is not pwm:%s !\n", name);
		return -1;
	}
	case PWM_IOC_RELEASE:
		return pwm2_release(arg);
	case PWM_IOC_DMA_DISABLE:
		return pwm2_dma_disable_loop(arg);
	case PWM_IOC_CONFIG: {
		struct pwm_ioc_config c;

		if (copy_from_user(&c, argp, sizeof(c)))
			goto copy_err;
		return pwm2_config(c.id, &c.cfg);
	}
	case PWM_IOC_SET_LEVEL: {
		int v[2];

		if (copy_from_user(v, argp, sizeof(v)))
			goto copy_err;
		return pwm2_set_level(v[0], v[1]);
	}
	case PWM_IOC_DMA_INIT: {
		struct pwm2_dma_config c;

		if (copy_from_user(&c, argp, sizeof(c)))
			goto copy_err;
		return pwm2_dma_init(c.id, &c);
	}
	case PWM_IOC_DMA_UPDATE: {
		struct pwm_ioc_dma d;
		struct pwm2_dma_data data;
		u32 *buf;

		if (copy_from_user(&d, argp, sizeof(d)))
			goto copy_err;
		buf = kmalloc(d.count * 4, GFP_KERNEL);
		if (!buf) {
			printk(KERN_ERR "PWM: pwm ch%d malloc dma data err\n", d.id);
			return -1;
		}
		if (copy_from_user(buf, (void __user *)d.buf, d.count * 4)) {
			kfree(buf);
			goto copy_err;
		}
		data.buf = buf;
		data.count = d.count;
		data.loop = d.loop;
		ret = pwm2_dma_update(d.id, &data);
		kfree(buf);
		return ret;
	}
	default:
		return -1;
	}

copy_err:
	printk(KERN_ERR "PWM: copy_from_user err!\n");
	return -1;
}

static int pwm_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pwm_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations pwm_fops = {
	.owner = THIS_MODULE,
	.open = pwm_open,
	.release = pwm_close,
	.unlocked_ioctl = pwm_ioctl,
};

static struct miscdevice pwm_mdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "jz_pwm",
	.fops = &pwm_fops,
};

// --- load -------------------------------------------------------------------

// The counter runs from EPLL through mux_pwm and div_pwm, both at the EPLL
// rate.
static int __init jz_pwm_init(void)
{
	struct clk *epll;
	int i, ret;

	pwm_mux_clk = clk_get(NULL, "mux_pwm");
	if (IS_ERR(pwm_mux_clk))
		return PTR_ERR(pwm_mux_clk);
	epll = clk_get(NULL, "epll");
	if (IS_ERR(epll)) {
		ret = PTR_ERR(epll);
		goto put_mux;
	}
	clk_src_rate = clk_get_rate(epll);
	clk_set_parent(pwm_mux_clk, epll);
	clk_prepare_enable(pwm_mux_clk);
	clk_put(epll);

	pwm_div_clk = clk_get(NULL, "div_pwm");
	if (IS_ERR(pwm_div_clk)) {
		ret = PTR_ERR(pwm_div_clk);
		goto stop_mux;
	}
	clk_set_rate(pwm_div_clk, clk_src_rate);
	clk_prepare_enable(pwm_div_clk);
	printk("pwm clk source rate %ld\n", clk_src_rate);

	pwm_gate_clk = clk_get(NULL, "gate_pwm");
	if (IS_ERR(pwm_gate_clk)) {
		ret = PTR_ERR(pwm_gate_clk);
		goto stop_div;
	}
	clk_prepare_enable(pwm_gate_clk);

	for (i = 0; i < PWM_CHANNELS; i++) {
		mutex_init(&pwmdata[i].lock);
		init_waitqueue_head(&pwmdata[i].dma_wait);
	}
	ret = misc_register(&pwm_mdevice);
	if (ret < 0)
		goto stop_gate;
	return 0;

stop_gate:
	clk_disable_unprepare(pwm_gate_clk);
	clk_put(pwm_gate_clk);
stop_div:
	clk_disable_unprepare(pwm_div_clk);
	clk_put(pwm_div_clk);
stop_mux:
	clk_disable_unprepare(pwm_mux_clk);
put_mux:
	clk_put(pwm_mux_clk);
	return ret;
}

static void __exit jz_pwm_exit(void)
{
	misc_deregister(&pwm_mdevice);
	clk_disable_unprepare(pwm_gate_clk);
	clk_disable_unprepare(pwm_div_clk);
	clk_disable_unprepare(pwm_mux_clk);
	clk_put(pwm_gate_clk);
	clk_put(pwm_div_clk);
	clk_put(pwm_mux_clk);
}

module_init(jz_pwm_init);
module_exit(jz_pwm_exit);

MODULE_DESCRIPTION("Ingenic SoC PWM driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
