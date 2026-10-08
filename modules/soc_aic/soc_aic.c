// SPDX-License-Identifier: GPL-2.0
//
// soc_aic -- the X1600 AIC (I2S controller) as the ASoC CPU DAI and platform
// "ingenic-aic": the DAI, its PCM over cyclic DMA, and the hooks the HiBy
// sound cards use when the AIC is the I2S slave.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock soc_aic.sh passes them):
//
//   aic_select_clk                       2: playback and capture share the
//                                        transmit clocks (symmetric rates)
//   aic_if_send_invalid_data             1: the first playback hw_params
//                                        plays a period of silence
//   aic_if_send_invalid_data_everytime   1: every playback hw_params does
//   aic_send_invalid_data_time_ms        how long that silence plays
//
// Playback: the DMA runs cyclically over the runtime buffer; an hrtimer reads
// the DMA position, zeroes what has been played and reports the period.
// Capture: the DMA runs cyclically over a separate buffer, and a kernel
// thread copies from it into the runtime buffer. Starting and stopping run
// on an ordered workqueue.
//
// Exported: aic_slave_trigger(start, stop), callbacks for the sound card,
// called with the sample rate around the start of playback when the codec
// side is the clock master.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/addrspace.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern int gpio_set_func(int gpio, int func);
extern char *gpio_to_str(int gpio, char *buf);
extern u64 local_clock_us(void);
extern void __assert(const char *expr, const char *file, int line, const char *func);

#define AIC_ASSERT(x) \
	do { if (!(x)) __assert(#x, __FILE__, __LINE__, __func__); } while (0)

// --- registers --------------------------------------------------------------

#define AIC_PHYS		0x10079000
#define AIC_REG(off)		((void __iomem *)CKSEG1ADDR(AIC_PHYS + (off)))
#define AICFR			0x00
#define AICCR			0x04
#define I2SCR			0x10
#define AICSR			0x14
#define I2SDIV			0x30
#define AICDR			0x34

#define AICFR_ENB		BIT(0)
#define AICFR_SYNCD		BIT(1)
#define AICFR_BCKD		BIT(2)
#define AICFR_RST		BIT(3)
#define AICFR_AUSEL		BIT(4)
#define AICFR_LSMP		BIT(6)
#define AICFR_DMODE		BIT(8)
#define AICFR_MSB		BIT(12)
#define AICFR_TFTH_MASK		(0x1f << 16)
#define AICFR_RFTH_MASK		(0xf << 24)

#define AICCR_EREC		BIT(0)
#define AICCR_ERPL		BIT(1)
#define AICCR_RFLUSH		BIT(7)
#define AICCR_TFLUSH		BIT(8)
#define AICCR_TDMS		BIT(14)
#define AICCR_RDMS		BIT(15)
#define AICCR_M2S		(3 << 12)
#define AICCR_ISS_MASK		(7 << 16)
#define AICCR_OSS_MASK		(7 << 19)
#define AICCR_CHANNEL_MASK	(7 << 24)
#define AICCR_PACK16		BIT(28)
// Cleared at every initialisation of the common settings.
#define AICCR_INIT_CLEAR	0x47f

#define I2SCR_AMSL		BIT(0)
#define I2SCR_INIT_CLEAR	(3 << 16)

#define AICSR_TFL(v)		(((v) >> 8) & 0x3f)
#define AICSR_RFL(v)		(((v) >> 24) & 0x3f)

#define I2SDIV_DV_MASK		0x1ff
#define I2SDIV_IDV_MASK		(0x1ff << 16)

// DMA request types of the AIC on the X1600 DMA controller.
#define AIC_DMA_TX		62
#define AIC_DMA_RX		63

// The I2S pins, port B.
#define PIN_RX_DATA		53
#define PIN_RX_MCLK		54
#define PIN_RX_BCLK		55
#define PIN_RX_LRCLK		56
#define PIN_TX_DATA		57
#define PIN_TX_MCLK		58
#define PIN_TX_BCLK		59
#define PIN_TX_LRCLK		60

// set_sysclk ids of the sound cards: MCLK output on and off.
#define AIC_SYSCLK_MCLK_ON	2
#define AIC_SYSCLK_MCLK_OFF	3

#define AIC_CLK_RATE		12288000
#define AIC_PREALLOC_BYTES	0x200000
#define AIC_CAPTURE_CHUNK	512

static int aic_select_clk;
static int aic_send_invalid_data_time_ms;
static int aic_if_send_invalid_data_everytime;
static int aic_if_send_invalid_data;
module_param(aic_select_clk, int, 0644);
module_param(aic_if_send_invalid_data, int, 0644);
module_param(aic_if_send_invalid_data_everytime, int, 0644);
module_param(aic_send_invalid_data_time_ms, int, 0644);

static void (*aic_slave_trigger_start)(void *data);
static void (*aic_slave_trigger_stop)(void *data);

// Clocks set to the MCLK rate at every hw_params: the i2s0t ones for
// playback, the i2s0r ones for capture.
static const char *const aic_tx_clk_table[] = {
	"mux_i2s0t", "div_i2s0t", "gate_i2s0t", "ce_i2s0t",
};

static const char *const aic_rx_clk_table[] = {
	"mux_i2s0r", "div_i2s0r", "gate_i2s0r", "ce_i2s0r",
};

struct aic_dma_param {
	int type;
	struct dma_chan *chan;
	dma_addr_t fifo;
	dma_addr_t buf;
	size_t len;
	size_t period;
	enum dma_slave_buswidth width;
	u32 burst;
};

// One direction. Positions are byte offsets. Playback: the DMA reads the
// runtime buffer (dma_area) and dma_pos is the hardware pointer. Capture:
// the DMA writes dma_area, a buffer of its own; the copy thread moves what
// it holds (avail bytes from read_pos) into the runtime buffer (area), and
// copy_pos is the hardware pointer.
struct aic_stream {
	u32 dma_pos;
	u32 read_pos;
	u32 copy_pos;
	u32 avail;
	u32 period_ns;
	u32 burst;
	u32 period_frames;
	u32 frame_bytes;
	u32 channels;
	u32 format;
	u32 div;
	void *dma_area;
	void *area;
	u32 buf_bytes;
	u32 dma_range;
	int busy;
	int running;
	struct aic_dma_param dma;
	struct hrtimer timer;
	struct mutex mutex;
	struct task_struct *thread;
	int stop;
	int exited;
	struct work_struct start_work;
	struct work_struct stop_work;
	struct snd_pcm_substream *substream;
};

static struct {
	struct device *dev;
	struct platform_device *pdev;
	struct aic_stream playback;
	struct aic_stream capture;
	int master;		// the AIC drives BCLK and LRCLK
	int left_justified;
	u32 rate;
	int sysclk_id;
	int sysclk_dir;
	u32 sysclk_freq;
	int mclk_en;
	u32 inited;		// BIT(0) playback, BIT(1) capture
	int enable_cnt;
	struct clk *gate;
	struct mutex mutex;
	struct workqueue_struct *wq;
} aic_dev;

static u32 gpio_status;

static inline u32 aic_read(unsigned int off)
{
	return readl(AIC_REG(off));
}

static inline void aic_write(unsigned int off, u32 v)
{
	writel(v, AIC_REG(off));
}

static inline void aic_update(unsigned int off, u32 mask, u32 v)
{
	aic_write(off, (aic_read(off) & ~mask) | (v & mask));
}

void aic_slave_trigger(void (*start)(void *data), void (*stop)(void *data))
{
	aic_slave_trigger_start = start;
	aic_slave_trigger_stop = stop;
}
EXPORT_SYMBOL(aic_slave_trigger);

static void aic_slave_start(void)
{
	if (!aic_dev.master && aic_slave_trigger_start)
		aic_slave_trigger_start((void *)(unsigned long)aic_dev.rate);
}

static void aic_slave_stop(void)
{
	if (!aic_dev.master && aic_slave_trigger_stop)
		aic_slave_trigger_stop((void *)(unsigned long)aic_dev.rate);
}

// --- pins and clocks --------------------------------------------------------

// Requests the pin and sets its function, once: a pin already held is left
// as it is.
static int m_gpio_request(int gpio, const char *name, int func)
{
	char buf[16];

	if ((gpio_status >> (gpio % 32)) & 1)
		return 0;
	if (gpio_request(gpio, name)) {
		pr_err("AIC: failed to request %s gpio: %s\n", name, gpio_to_str(gpio, buf));
		return -EINVAL;
	}
	gpio_set_func(gpio, func);
	gpio_status |= 1 << (gpio % 32);
	return 0;
}

static void m_gpio_free(int gpio)
{
	if (gpio < 0 || !((gpio_status >> (gpio & 31)) & 1))
		return;
	gpio_free(gpio);
	gpio_status &= ~(1 << (gpio & 31));
}

static void aic_clk_start(const char *name, unsigned long rate)
{
	struct clk *clk = clk_get(NULL, name);

	BUG_ON(IS_ERR(clk));
	clk_set_rate(clk, rate);
	clk_prepare_enable(clk);
	clk_put(clk);
}

static void aic_clk_stop(const char *name)
{
	struct clk *clk = clk_get(NULL, name);

	BUG_ON(IS_ERR(clk));
	clk_disable_unprepare(clk);
	clk_put(clk);
}

// --- the controller ---------------------------------------------------------

static void aic_enable(void)
{
	if (!aic_dev.enable_cnt++)
		aic_update(AICFR, AICFR_ENB, AICFR_ENB);
}

static void aic_disable(void)
{
	if (!--aic_dev.enable_cnt)
		aic_update(AICFR, AICFR_ENB, 0);
}

static void aic_hal_init(void)
{
	aic_update(AICFR, AICFR_ENB, 0);
	aic_update(AICCR, AICCR_TDMS, 0);
	aic_update(AICCR, AICCR_TFLUSH, AICCR_TFLUSH);
	aic_update(AICCR, AICCR_ERPL, 0);
}

// Waits for the transmit FIFO to drain, flushing it, and says so once after
// 16 ms.
static void aic_wait_tx_empty(bool flush)
{
	int tries = 15;

	while (AICSR_TFL(aic_read(AICSR))) {
		if (flush)
			aic_update(AICCR, AICCR_TFLUSH, AICCR_TFLUSH);
		udelay(1000);
		if (!tries) {
			printk("AIC: failed to wait tx fifo empty: %x\n", aic_read(AICSR));
			tries = -1;
		}
		tries--;
	}
}

static void aic_hal_start_playback(void)
{
	aic_wait_tx_empty(true);
	aic_update(AICCR, AICCR_TDMS, AICCR_TDMS);
	aic_update(AICCR, AICCR_ERPL, AICCR_ERPL);
	aic_enable();
}

static void aic_stop_playback(void)
{
	mutex_lock(&aic_dev.mutex);
	aic_disable();
	aic_update(AICCR, AICCR_TDMS, 0);
	aic_update(AICCR, AICCR_TFLUSH, AICCR_TFLUSH);
	aic_wait_tx_empty(false);
	aic_update(AICCR, AICCR_ERPL, 0);
	aic_dev.playback.dma_pos = 0;
	snd_pcm_period_elapsed(aic_dev.playback.substream);
	mutex_unlock(&aic_dev.mutex);
}

// Format, clock directions and FIFO thresholds, then a reset of the AIC. A
// reset that does not finish in 100 ms means no bit clock: the codec is not
// running.
static void aic_hal_init_common_setting(void)
{
	u32 fr = aic_read(AICFR);
	u64 t0;

	fr = (fr & ~AICFR_TFTH_MASK) | (0x10 << 16);
	fr = (fr & ~AICFR_RFTH_MASK) | (0x3 << 24);
	fr &= ~(AICFR_MSB | AICFR_LSMP);
	fr |= AICFR_AUSEL;
	if (aic_select_clk == 2)
		fr &= ~AICFR_DMODE;
	else
		fr |= AICFR_DMODE;
	fr = (fr & ~AICFR_BCKD) | (aic_dev.master << 2 & AICFR_BCKD);
	fr = (fr & ~AICFR_SYNCD) | (aic_dev.master << 1 & AICFR_SYNCD);
	fr &= ~AICFR_ENB;
	aic_write(AICFR, fr);

	aic_update(AICFR, AICFR_RST, AICFR_RST);
	t0 = local_clock_us();
	while (aic_read(AICFR) & AICFR_RST) {
		if (local_clock_us() - t0 >= 100001)
			panic(aic_dev.sysclk_id ? "AIC:reset aic failure\n" :
			      "AIC:Please enable external codec before enable aic\n");
	}

	aic_write(AICCR, aic_read(AICCR) & ~AICCR_INIT_CLEAR);
	aic_write(I2SCR, (aic_read(I2SCR) & ~I2SCR_INIT_CLEAR & ~I2SCR_AMSL) |
		  (aic_dev.left_justified & I2SCR_AMSL));
}

static void aic_init_common_setting(struct aic_stream *s)
{
	if (!aic_dev.inited)
		aic_hal_init_common_setting();
	aic_dev.inited |= s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? BIT(0) : BIT(1);
}

// Sample size field of AICCR for a PCM format.
static u32 aic_sample_size(u32 format)
{
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		return 1;
	case SNDRV_PCM_FORMAT_S24_LE:
		return 4;
	default:
		return 0;
	}
}

static void aic_init_playback_setting(struct aic_stream *s)
{
	bool stereo = s->channels == 2;
	u32 cr;

	if (aic_dev.master)
		aic_write(I2SDIV, s->div & I2SDIV_DV_MASK);
	cr = aic_read(AICCR);
	cr = (cr & ~AICCR_PACK16) |
	     (stereo && s->format == SNDRV_PCM_FORMAT_S16_LE ? AICCR_PACK16 : 0);
	cr = (cr & ~AICCR_CHANNEL_MASK) | (stereo ? 1 << 24 : 0);
	cr = (cr & ~AICCR_OSS_MASK) | (aic_sample_size(s->format) << 19);
	aic_write(AICCR, cr);
}

static void aic_init_capture_setting(struct aic_stream *s)
{
	u32 cr;

	if (aic_dev.master) {
		if (aic_select_clk == 2)
			aic_write(I2SDIV, s->div & I2SDIV_DV_MASK);
		else
			aic_write(I2SDIV, s->div << 16 & I2SDIV_IDV_MASK);
	}
	cr = aic_read(AICCR);
	cr = (cr & ~AICCR_ISS_MASK) | (aic_sample_size(s->format) << 16);
	cr = (cr & ~AICCR_M2S) | (s->channels != 2 ? 1 << 12 : 0);
	aic_write(AICCR, cr);
}

// --- DMA --------------------------------------------------------------------

static int dma_param_config(struct aic_dma_param *p, enum dma_transfer_direction dir)
{
	struct dma_slave_config cfg = {
		.direction = dir,
		.src_addr_width = p->width,
		.dst_addr_width = p->width,
		.src_maxburst = p->burst,
		.dst_maxburst = p->burst,
	};

	if (dir == DMA_DEV_TO_MEM) {
		cfg.src_addr = p->fifo;
		cfg.slave_id = AIC_DMA_RX;
	} else {
		cfg.dst_addr = p->fifo;
		cfg.slave_id = AIC_DMA_TX;
	}
	if (dmaengine_slave_config(p->chan, &cfg)) {
		pr_err("AIC: Failed to config dma chan\n");
		return -1;
	}
	return 0;
}

static int dma_param_submit_cyclic(struct aic_dma_param *p, enum dma_transfer_direction dir)
{
	struct dma_async_tx_descriptor *desc;
	dma_addr_t buf = p->buf;
	size_t len = p->len, period = p->period;

	if (dma_param_config(p, dir) < 0)
		return -1;
	desc = dmaengine_prep_dma_cyclic(p->chan, buf, len, period, dir, DMA_CTRL_ACK);
	if (!desc) {
		pr_err("AIC: Failed to prepare dma desc\n");
		return -1;
	}
	desc->callback = NULL;
	desc->callback_param = p;
	dmaengine_submit(desc);
	dma_async_issue_pending(p->chan);
	return 0;
}

static void aic_dma_submit_cyclic(struct aic_stream *s, enum dma_transfer_direction dir)
{
	s->dma.buf = virt_to_phys(s->dma_area);
	s->dma.len = s->buf_bytes;
	s->dma.period = s->period_frames;
	dma_param_submit_cyclic(&s->dma, dir);
}

// A cyclic transfer of len bytes at virt, KSEG0.
static int aic_dma_invalid_data_cyclic(struct aic_dma_param *p, enum dma_transfer_direction dir,
				       void *virt, size_t len)
{
	struct dma_async_tx_descriptor *desc;

	if (dma_param_config(p, dir) < 0)
		return -1;
	desc = dmaengine_prep_dma_cyclic(p->chan, virt_to_phys(virt), len, len, dir, DMA_CTRL_ACK);
	if (!desc) {
		pr_err("AIC: Failed to prepare dma desc\n");
		return -1;
	}
	desc->callback = NULL;
	desc->callback_param = NULL;
	dmaengine_submit(desc);
	dma_async_issue_pending(p->chan);
	return 0;
}

static void aic_dma_terminate(struct aic_stream *s)
{
	dmaengine_terminate_all(s->dma.chan);
}

// The address the DMA is at, through the Ingenic legacy hook; read twice
// before giving up on one outside [start, start + size).
static dma_addr_t get_dma_addr(struct dma_chan *chan, dma_addr_t start, size_t size,
			       enum dma_transfer_direction dir)
{
	dma_addr_t addr = 0;
	int tries;

	for (tries = 0; tries < 2; tries++) {
		addr = chan->device->get_current_trans_addr(chan, NULL, NULL, dir);
		if (addr >= start && addr < start + size)
			return addr;
	}
	printk(KERN_WARNING "AIC: DMA address[0x%08x] range:[%08x, %08x]  illegal!\n",
	       addr, start, start + size);
	return start;
}

// Coherent memory as a KSEG0 address.
static void *m_dma_alloc_coherent(size_t size)
{
	dma_addr_t handle;
	void *mem = dma_alloc_coherent(NULL, size, &handle, GFP_KERNEL);

	AIC_ASSERT(mem);
	return (void *)CKSEG0ADDR(mem);
}

static void m_dma_free_coherent(void *virt, size_t size)
{
	dma_free_coherent(NULL, size, (void *)CKSEG1ADDR(virt), virt_to_phys(virt));
}

// --- playback timer and capture thread --------------------------------------

// Every period_ns: zeroes what the DMA has played since the last tick and
// reports a period.
static enum hrtimer_restart aic_hrtimer_callback(struct hrtimer *timer)
{
	struct aic_stream *s = &aic_dev.playback;
	dma_addr_t phys = virt_to_phys(s->dma_area);
	u32 old = s->dma_pos, pos;

	hrtimer_forward(timer, hrtimer_get_expires(timer), ns_to_ktime(s->period_ns));
	pos = get_dma_addr(s->dma.chan, phys, s->dma_range, DMA_MEM_TO_DEV) - phys;
	if (pos == old)
		return HRTIMER_RESTART;
	s->dma_pos = pos;
	if (!pos)
		return HRTIMER_RESTART;
	if ((int)old < (int)pos) {
		memset(s->dma_area + old, 0, pos - old);
		dma_cache_sync(NULL, s->dma_area + old, pos - old, DMA_TO_DEVICE);
	} else {
		memset(s->dma_area + old, 0, s->buf_bytes - old);
		dma_cache_sync(NULL, s->dma_area + old, s->buf_bytes - old, DMA_TO_DEVICE);
		memset(s->dma_area, 0, pos);
		dma_cache_sync(NULL, s->dma_area, pos, DMA_TO_DEVICE);
	}
	snd_pcm_period_elapsed(s->substream);
	return HRTIMER_RESTART;
}

static void do_memcpy(void *dst, void *src, size_t n)
{
	dma_cache_sync(NULL, src, n, DMA_FROM_DEVICE);
	memcpy(dst, src, n);
}

// Moves at most AIC_CAPTURE_CHUNK bytes per round from the DMA buffer to
// the runtime buffer, keeping one burst back; sleeps a timer period when a
// round comes up short. A DMA that catches up with the read position
// moves it to the burst after the DMA's.
static int capture_dma_copy_thread(void *data)
{
	struct aic_stream *s = data;
	u32 buf_bytes = s->dma_range;
	u32 sleep_us = s->period_ns / 1000;
	u32 chunk, left, copied, hw, delta, n, end, size;
	dma_addr_t phys;
	void *dst;

	s->copy_pos = 0;
	s->exited = 0;
	while (!s->stop) {
		mutex_lock(&s->mutex);
		chunk = min_t(int, buf_bytes - s->copy_pos, AIC_CAPTURE_CHUNK);
		dst = s->area + s->copy_pos;
		left = chunk;
		copied = 0;
		while (left) {
			phys = virt_to_phys(s->dma_area);
			size = s->buf_bytes;
			hw = get_dma_addr(s->dma.chan, phys, size, DMA_DEV_TO_MEM) - phys;
			delta = (size - s->dma_pos + hw) % size;
			s->dma_pos = hw;
			s->avail += delta;
			if (s->avail > size - s->burst) {
				s->read_pos = (((s->burst - 1 + hw) & -s->burst) + s->burst) % size;
				s->avail = size - (size - hw + s->read_pos) % size;
			}
			if (s->avail <= s->burst)
				break;
			n = min(s->avail - s->burst, left);
			end = s->read_pos + n;
			size = s->buf_bytes;
			if (end > size) {
				do_memcpy(dst, s->dma_area + s->read_pos, size - s->read_pos);
				do_memcpy(dst + size - s->read_pos, s->dma_area, n - (size - s->read_pos));
			} else {
				do_memcpy(dst, s->dma_area + s->read_pos, n);
			}
			copied += n;
			dma_cache_sync(NULL, dst, n, DMA_TO_DEVICE);
			s->read_pos = end % size;
			dst += n;
			left -= n;
			s->avail -= n;
		}
		s->copy_pos += copied;
		if (s->copy_pos >= buf_bytes)
			s->copy_pos = 0;
		if (copied)
			snd_pcm_period_elapsed(s->substream);
		mutex_unlock(&s->mutex);
		if (copied != chunk)
			usleep_range(sleep_us, sleep_us);
	}
	s->exited = 1;
	return 0;
}

// --- start and stop work ----------------------------------------------------

static void start_work(struct work_struct *work)
{
	struct aic_stream *s = container_of(work, struct aic_stream, start_work);
	struct task_struct *t;

	if (!s->running)
		return;
	s->busy = 1;

	if (s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mutex_lock(&s->mutex);
		aic_slave_start();
		aic_init_common_setting(s);
		aic_init_playback_setting(s);
		aic_slave_stop();
		aic_dma_submit_cyclic(&aic_dev.playback, DMA_MEM_TO_DEV);
		hrtimer_forward_now(&aic_dev.playback.timer, ns_to_ktime(aic_dev.playback.period_ns));
		hrtimer_start_expires(&aic_dev.playback.timer, HRTIMER_MODE_ABS);
		mutex_lock(&aic_dev.mutex);
		aic_hal_start_playback();
		mutex_unlock(&aic_dev.mutex);
		aic_slave_start();
		mutex_unlock(&s->mutex);
		return;
	}

	mutex_lock(&s->mutex);
	mutex_lock(&aic_dev.mutex);
	aic_enable();
	aic_update(AICCR, AICCR_RFLUSH, AICCR_RFLUSH);
	aic_update(AICCR, AICCR_EREC, AICCR_EREC);
	{
		u64 t0 = local_clock_us();

		// Waits for the first samples; after 10 ms it complains at every
		// look until they come.
		while (!AICSR_RFL(aic_read(AICSR))) {
			if (local_clock_us() - t0 < 10001)
				continue;
			if (!AICSR_RFL(aic_read(AICSR)))
				printk("AIC: failed to wait rx fifo empty: %x\n", aic_read(AICSR));
		}
	}
	aic_update(AICCR, AICCR_RDMS, AICCR_RDMS);
	mutex_unlock(&aic_dev.mutex);
	aic_dma_submit_cyclic(&aic_dev.capture, DMA_DEV_TO_MEM);
	aic_dev.capture.exited = 1;
	t = kthread_create(capture_dma_copy_thread, &aic_dev.capture, "capture_dma_copy");
	aic_dev.capture.thread = t;
	if (t && !IS_ERR(t))
		wake_up_process(t);
	mutex_unlock(&s->mutex);
}

static void stop_work(struct work_struct *work)
{
	struct aic_stream *s = container_of(work, struct aic_stream, stop_work);

	if (s->running == 1)
		return;

	if (s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mutex_lock(&s->mutex);
		aic_slave_stop();
		hrtimer_cancel(&aic_dev.playback.timer);
		udelay(1000);
		aic_dma_terminate(&aic_dev.playback);
		aic_stop_playback();
		aic_dev.inited &= ~BIT(0);
		mutex_unlock(&s->mutex);
		s->busy = 0;
		return;
	}

	s->stop = 1;
	while (!aic_dev.capture.exited)
		usleep_range(300, 300);
	aic_dev.capture.thread = NULL;
	mutex_lock(&s->mutex);
	aic_dma_terminate(&aic_dev.capture);
	mutex_lock(&aic_dev.mutex);
	aic_update(AICCR, AICCR_RDMS, 0);
	aic_update(AICCR, AICCR_EREC, 0);
	aic_update(AICCR, AICCR_RFLUSH, AICCR_RFLUSH);
	aic_disable();
	mutex_unlock(&aic_dev.mutex);
	mutex_unlock(&s->mutex);
	s->busy = 0;
}

// --- DAI --------------------------------------------------------------------

static int aic_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	aic_dev.sysclk_id = clk_id;
	aic_dev.sysclk_freq = freq;
	aic_dev.sysclk_dir = dir;
	if (clk_id == AIC_SYSCLK_MCLK_ON)
		aic_dev.mclk_en = 1;
	else if (clk_id == AIC_SYSCLK_MCLK_OFF)
		aic_dev.mclk_en = 0;
	return 0;
}

static int aic_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aic_dev.left_justified = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		aic_dev.left_justified = 1;
		break;
	default:
		pr_err("AIC: fmt error: %x", fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic_dev.master = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic_dev.master = 1;
		break;
	default:
		pr_err("AIC: clk dir error: %x", fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}
	return 0;
}

// MCLK when the sound card set none: 768, 512, 384 or 256 times the rate.
static u32 aic_default_mclk(u32 rate)
{
	if (rate <= 16000)
		return rate * 768;
	if (rate <= 24000)
		return rate * 512;
	if (rate <= 32000)
		return rate * 384;
	return rate * 256;
}

static int aic_request_tx_pins(void)
{
	if (m_gpio_request(PIN_TX_MCLK, "i2s-tx-mclk", GPIO_OUTPUT0) < 0)
		return -EINVAL;
	if (m_gpio_request(PIN_TX_BCLK, "i2s-tx-bclk", GPIO_FUNC_0) < 0)
		goto free_mclk;
	if (m_gpio_request(PIN_TX_LRCLK, "i2s-tx-LR-clk", GPIO_FUNC_0) < 0)
		goto free_bclk;
	if (m_gpio_request(PIN_TX_DATA, "i2s-tx-data-out", GPIO_FUNC_0) < 0)
		goto free_lrclk;
	return 0;

free_lrclk:
	m_gpio_free(PIN_TX_LRCLK);
free_bclk:
	m_gpio_free(PIN_TX_BCLK);
free_mclk:
	m_gpio_free(PIN_TX_MCLK);
	return -EINVAL;
}

// Capture pins by aic_select_clk: 2 the transmit clocks, 1 the transmit
// MCLK with the receive clocks, 0 the receive clocks; then the data pin.
static int aic_request_rx_pins(void)
{
	int sel = aic_select_clk;

	if (sel == 2) {
		if (m_gpio_request(PIN_TX_MCLK, "i2s-tx-mclk", GPIO_OUTPUT0) < 0)
			return -EINVAL;
		if (m_gpio_request(PIN_TX_BCLK, "i2s-tx-bclk", GPIO_FUNC_0) < 0 ||
		    m_gpio_request(PIN_TX_LRCLK, "i2s-tx-lr_clk", GPIO_FUNC_0) < 0)
			goto fail;
	} else if (sel == 1) {
		if (m_gpio_request(PIN_TX_MCLK, "i2s-tx-mclk", GPIO_OUTPUT0) < 0)
			return -EINVAL;
		if (m_gpio_request(PIN_RX_BCLK, "i2s-rx-bclk", GPIO_FUNC_0) < 0 ||
		    m_gpio_request(PIN_RX_LRCLK, "i2s-rx-LR-clk", GPIO_FUNC_0) < 0)
			goto fail;
	} else if (sel == 0) {
		if (m_gpio_request(PIN_RX_MCLK, "i2s-rx-mclk", GPIO_FUNC_0) < 0)
			return -EINVAL;
		if (m_gpio_request(PIN_RX_BCLK, "i2s-rx-bclk", GPIO_FUNC_0) < 0 ||
		    m_gpio_request(PIN_RX_LRCLK, "i2s-rx-LR-clk", GPIO_FUNC_0) < 0)
			goto fail;
	}
	if (m_gpio_request(PIN_RX_DATA, "i2s-rx-data-in", GPIO_FUNC_0) < 0)
		goto fail;
	return 0;

fail:
	m_gpio_free(PIN_RX_DATA);
	if (sel == 2) {
		m_gpio_free(PIN_TX_MCLK);
		m_gpio_free(PIN_TX_BCLK);
		m_gpio_free(PIN_TX_LRCLK);
	} else if (sel == 1) {
		m_gpio_free(PIN_TX_MCLK);
		m_gpio_free(PIN_RX_BCLK);
		m_gpio_free(PIN_RX_LRCLK);
	} else if (sel == 0) {
		m_gpio_free(PIN_RX_MCLK);
		m_gpio_free(PIN_RX_BCLK);
		m_gpio_free(PIN_RX_LRCLK);
	}
	return -EINVAL;
}

// Rate, format and clocks of one direction. Playback and capture run at one
// rate. The AIC registers are set here for capture; for playback when the
// stream starts.
static int aic_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	const char *const *clks = playback ? aic_tx_clk_table : aic_rx_clk_table;
	struct aic_stream *s;
	u32 rate = params_rate(params);
	u32 mclk, div;
	int i, ret;

	mutex_lock(&aic_dev.mutex);
	if (playback && (aic_dev.inited & BIT(0))) {
		pr_err("AIC: playback is already initialized.\n");
		ret = -EBUSY;
		goto out;
	}
	if (!playback && (aic_dev.inited & BIT(1))) {
		pr_err("AIC: caputer is already initialized.\n");
		ret = -EBUSY;
		goto out;
	}
	s = playback ? &aic_dev.playback : &aic_dev.capture;
	if (aic_dev.inited && rate != aic_dev.rate) {
		pr_err("AIC: capture and playback sample rate %d are different!\n", aic_dev.rate);
		ret = -EINVAL;
		goto out;
	}

	s->channels = params_channels(params);
	s->format = params_format(params);
	s->substream = substream;
	aic_dev.rate = rate;
	mclk = aic_dev.sysclk_freq ? aic_dev.sysclk_freq : aic_default_mclk(rate);
	div = (mclk / rate) >> 6;
	if (div & 1)
		div--;
	s->div = div;
	for (i = 0; i < ARRAY_SIZE(aic_tx_clk_table); i++)
		aic_clk_start(clks[i], mclk);

	ret = 0;
	if (playback) {
		if (aic_request_tx_pins() < 0)
			ret = -EINVAL;
		else if (aic_dev.mclk_en == 1)
			gpio_set_func(PIN_TX_MCLK, GPIO_FUNC_0);
		goto out;
	}

	aic_init_common_setting(s);
	if (aic_request_rx_pins() < 0) {
		ret = -EINVAL;
		goto out;
	}
	if (aic_select_clk && aic_dev.mclk_en == 1)
		gpio_set_func(PIN_TX_MCLK, GPIO_FUNC_0);
	aic_init_capture_setting(s);
out:
	mutex_unlock(&aic_dev.mutex);
	return ret;
}

static int aic_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	mutex_lock(&aic_dev.mutex);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic_dev.inited &= ~BIT(0);
	else
		aic_dev.inited &= ~BIT(1);
	if (!aic_dev.mclk_en)
		gpio_direction_output(PIN_TX_MCLK, 0);
	mutex_unlock(&aic_dev.mutex);
	return 0;
}

// Starting and stopping need to sleep: both go to the workqueue.
static int aic_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct aic_stream *s = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			       &aic_dev.playback : &aic_dev.capture;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		s->running = 1;
		queue_work(aic_dev.wq, &s->start_work);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		s->running = 0;
		queue_work(aic_dev.wq, &s->stop_work);
		return 0;
	default:
		return -EINVAL;
	}
}

static int aic_dai_probe(struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops aic_dai_ops = {
	.set_sysclk = aic_set_sysclk,
	.set_fmt = aic_set_dai_fmt,
	.hw_params = aic_hw_params,
	.hw_free = aic_free,
	.trigger = aic_trigger,
};

// 8 to 768 kHz without 5512, 11025, 22050 and 64000 Hz.
#define AIC_RATES		0x1feea
#define AIC_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver aic_dai = {
	.probe = aic_dai_probe,
	.ops = &aic_dai_ops,
	.playback = {
		.formats = AIC_FORMATS,
		.rates = AIC_RATES,
		.channels_min = 1,
		.channels_max = 2,
	},
	.capture = {
		.formats = AIC_FORMATS,
		.rates = AIC_RATES,
		.channels_min = 1,
		.channels_max = 2,
	},
};

static const struct snd_soc_component_driver aic_component = {
	.name = "ingenic-aic-component",
};

// --- PCM --------------------------------------------------------------------

static const struct snd_pcm_hardware aic_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = AIC_FORMATS,
	.rates = AIC_RATES,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = AIC_PREALLOC_BYTES,
	.period_bytes_min = 0x1000,
	.period_bytes_max = AIC_PREALLOC_BYTES,
	.periods_min = 4,
	.periods_max = 0x200,
};

static int aic_dma_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	ret = snd_soc_set_runtime_hwparams(substream, &aic_pcm_hardware);
	if (ret) {
		pr_err("AIC: snd_soc_set_runtime_hwparams failed ret = %d\n", ret);
		return ret;
	}
	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret) {
		pr_err("AIC: align hw_param buffer failed ret = %d\n", ret);
		return ret;
	}
	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret) {
		pr_err("AIC: align hw_param period failed ret = %d\n", ret);
		return ret;
	}
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		pr_err("AIC: snd_pcm_hw_constraint_integer failed ret = %d\n", ret);
		return ret;
	}
	return 0;
}

static int aic_dma_pcm_close(struct snd_pcm_substream *substream)
{
	return 0;
}

// Plays aic_send_invalid_data_time_ms of silence, one period long, to
// settle the line before the first playback.
static void aic_send_invalid_data(struct aic_stream *s)
{
	void *mem = m_dma_alloc_coherent(s->period_frames);

	memset(mem, 0, s->period_frames);
	dma_cache_sync(NULL, mem, s->period_frames, DMA_TO_DEVICE);
	mutex_lock(&s->mutex);
	mutex_lock(&aic_dev.mutex);
	aic_hal_start_playback();
	mutex_unlock(&aic_dev.mutex);
	aic_dma_invalid_data_cyclic(&s->dma, DMA_MEM_TO_DEV, mem, s->period_frames);
	mutex_unlock(&s->mutex);
	usleep_range(aic_send_invalid_data_time_ms * 1000, aic_send_invalid_data_time_ms * 1000);
	mutex_lock(&s->mutex);
	aic_dma_terminate(s);
	aic_stop_playback();
	mutex_unlock(&s->mutex);
	m_dma_free_coherent(mem, s->period_frames);
	if (!aic_if_send_invalid_data_everytime)
		aic_if_send_invalid_data = 0;
}

// period_frames, the period size in frames, is also the DMA period in bytes;
// the timer ticks every period_frames bytes' worth of audio, in whole
// milliseconds.
static int aic_dma_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct aic_stream *s = playback ? &aic_dev.playback : &aic_dev.capture;
	u32 buffer_bytes = params_buffer_bytes(params);
	int ret;

	s->period_frames = params_period_size(params);
	if (s->format == SNDRV_PCM_FORMAT_S24_LE ||
	    (s->format == SNDRV_PCM_FORMAT_S16_LE && playback && s->channels != 1)) {
		s->dma.width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		s->dma.burst = 32;
	} else if (s->format == SNDRV_PCM_FORMAT_S16_LE && playback) {
		s->dma.width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		s->dma.burst = 16;
	} else if (s->format == SNDRV_PCM_FORMAT_S16_LE) {
		s->dma.width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		s->dma.burst = 2;
	}
	if (buffer_bytes % s->dma.burst) {
		pr_err("AIC: ERROR buf_size UNALIGN %d.\n", buffer_bytes);
		return -EINVAL;
	}
	s->burst = s->dma.burst;
	s->dma_range = buffer_bytes;
	s->buf_bytes = buffer_bytes;
	s->frame_bytes = snd_pcm_format_physical_width(params_format(params)) * (int)s->channels / 8;

	substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV;
	ret = snd_pcm_lib_malloc_pages(substream, buffer_bytes);
	if (ret < 0)
		return ret;

	if (playback) {
		if (aic_if_send_invalid_data)
			aic_send_invalid_data(s);
		s->dma_area = (void *)CKSEG0ADDR(substream->dma_buffer.area);
	} else {
		s->area = (void *)CKSEG0ADDR(substream->dma_buffer.area);
		s->dma_area = m_dma_alloc_coherent(s->buf_bytes);
	}

	s->period_ns = s->period_frames * 1000 / aic_dev.rate / s->frame_bytes * 1000000;
	s->dma_pos = 0;
	s->read_pos = 0;
	s->avail = 0;
	s->stop = 0;
	s->substream = substream;
	return 0;
}

static int aic_dma_pcm_hw_free(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		int tries = 1000;

		while (aic_dev.playback.busy) {
			usleep_range(600, 600);
			if (!tries) {
				printk("aic_dma_pcm_hw_free timeout\n");
				tries = -1;
			}
			tries--;
		}
	} else {
		while (aic_dev.capture.busy)
			usleep_range(600, 600);
		if (aic_dev.capture.dma_area) {
			m_dma_free_coherent(aic_dev.capture.dma_area, aic_dev.capture.buf_bytes);
			aic_dev.capture.dma_area = NULL;
		}
	}
	return snd_pcm_lib_free_pages(substream);
}

static int aic_dma_pcm_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static snd_pcm_uframes_t aic_dma_pcm_pointer(struct snd_pcm_substream *substream)
{
	u32 pos = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		  aic_dev.playback.dma_pos : aic_dev.capture.copy_pos;

	return bytes_to_frames(substream->runtime, pos);
}

// Playback maps the runtime buffer uncached.
static int aic_dma_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return snd_pcm_lib_default_mmap(substream, vma);
	vma->vm_pgoff = ((virt_to_phys(aic_dev.playback.dma_area) & PAGE_MASK) +
			 (vma->vm_pgoff << PAGE_SHIFT)) >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start,
			    vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static const struct snd_pcm_ops aic_dma_pcm_ops = {
	.open = aic_dma_pcm_open,
	.close = aic_dma_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = aic_dma_pcm_hw_params,
	.hw_free = aic_dma_pcm_hw_free,
	.prepare = aic_dma_pcm_prepare,
	.pointer = aic_dma_pcm_pointer,
	.mmap = aic_dma_pcm_mmap,
};

static int aic_dma_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	dma_cap_mask_t mask;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);
	aic_dev.playback.dma.chan = dma_request_channel(mask, NULL, &aic_dev.playback.dma);
	if (!aic_dev.playback.dma.chan)
		panic("AIC: aic dma tx_chan requested failed.\n");
	aic_dev.capture.dma.chan = dma_request_channel(mask, NULL, &aic_dev.capture.dma);
	if (!aic_dev.capture.dma.chan)
		panic("AIC: aic dma rx_chan requested failed.\n");
	ret = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, &aic_dev.pdev->dev,
						    AIC_PREALLOC_BYTES, AIC_PREALLOC_BYTES);
	if (ret)
		panic("AIC: aic preallocate mem failed ret = %d\n", ret);
	return 0;
}

static void aic_dma_pcm_free(struct snd_pcm *pcm)
{
	dma_release_channel(aic_dev.playback.dma.chan);
	dma_release_channel(aic_dev.capture.dma.chan);
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static struct snd_soc_platform_driver pcm_platform_driver = {
	.pcm_new = aic_dma_pcm_new,
	.pcm_free = aic_dma_pcm_free,
	.ops = &aic_dma_pcm_ops,
};

// --- platform device --------------------------------------------------------

static int aic_probe(struct platform_device *pdev)
{
	struct clk *gate;
	int i, ret;

	gate = clk_get(NULL, "gate_audio");
	aic_dev.gate = gate;
	BUG_ON(IS_ERR(gate));
	clk_prepare_enable(gate);
	for (i = 0; i < ARRAY_SIZE(aic_tx_clk_table); i++) {
		aic_clk_start(aic_tx_clk_table[i], AIC_CLK_RATE);
		aic_clk_start(aic_rx_clk_table[i], AIC_CLK_RATE);
	}

	aic_dev.dev = &pdev->dev;
	platform_set_drvdata(pdev, NULL);
	aic_dev.pdev = pdev;
	aic_dev.playback.dma.type = AIC_DMA_TX;
	aic_dev.capture.dma.type = AIC_DMA_RX;
	aic_dev.playback.dma.fifo = AIC_PHYS + AICDR;
	aic_dev.capture.dma.fifo = AIC_PHYS + AICDR;
	hrtimer_init(&aic_dev.playback.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aic_dev.playback.timer.function = aic_hrtimer_callback;
	hrtimer_init(&aic_dev.capture.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aic_dev.capture.timer.function = aic_hrtimer_callback;
	INIT_WORK(&aic_dev.playback.start_work, start_work);
	INIT_WORK(&aic_dev.capture.start_work, start_work);
	INIT_WORK(&aic_dev.playback.stop_work, stop_work);
	INIT_WORK(&aic_dev.capture.stop_work, stop_work);
	aic_dev.wq = create_singlethread_workqueue("aic_work");
	mutex_init(&aic_dev.mutex);
	mutex_init(&aic_dev.capture.mutex);
	mutex_init(&aic_dev.playback.mutex);
	if (aic_select_clk == 2)
		aic_dai.symmetric_rates = 1;

	// Function 0 on the transmit clock and data pins, then released.
	if (m_gpio_request(PIN_TX_BCLK, "i2s-tx-bclk", GPIO_FUNC_0) >= 0)
		m_gpio_free(PIN_TX_BCLK);
	if (m_gpio_request(PIN_TX_LRCLK, "i2s-tx-LR-clk", GPIO_FUNC_0) >= 0)
		m_gpio_free(PIN_TX_LRCLK);
	if (m_gpio_request(PIN_TX_DATA, "i2s-tx-data-out", GPIO_FUNC_0) >= 0)
		m_gpio_free(PIN_TX_DATA);
	aic_hal_init();

	ret = snd_soc_register_component(&pdev->dev, &aic_component, &aic_dai, 1);
	if (ret)
		panic("AIC: aic snd_soc_register_component failed ret = %d!\n", ret);
	ret = snd_soc_register_platform(&pdev->dev, &pcm_platform_driver);
	if (ret)
		panic("AIC: aic snd_soc_register_platform failed ret = %d!\n", ret);
	return 0;
}

static int aic_remove(struct platform_device *pdev)
{
	int i;

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	destroy_workqueue(aic_dev.wq);
	m_gpio_free(PIN_RX_DATA);
	m_gpio_free(PIN_RX_LRCLK);
	m_gpio_free(PIN_RX_BCLK);
	m_gpio_free(PIN_RX_MCLK);
	m_gpio_free(PIN_TX_DATA);
	m_gpio_free(PIN_TX_LRCLK);
	m_gpio_free(PIN_TX_BCLK);
	m_gpio_free(PIN_TX_MCLK);
	gpio_status = 0;
	for (i = 0; i < ARRAY_SIZE(aic_tx_clk_table); i++) {
		aic_clk_stop(aic_tx_clk_table[i]);
		aic_clk_stop(aic_rx_clk_table[i]);
	}
	clk_disable_unprepare(aic_dev.gate);
	clk_put(aic_dev.gate);
	return 0;
}

static void aic_device_release(struct device *dev)
{
}

static struct platform_device aic_device = {
	.name = "ingenic-aic",
	.id = PLATFORM_DEVID_NONE,
	.dev = {
		.release = aic_device_release,
	},
};

static struct platform_driver aic_driver = {
	.probe = aic_probe,
	.remove = aic_remove,
	.driver = {
		.name = "ingenic-aic",
		.owner = THIS_MODULE,
	},
};

static int __init aic_init(void)
{
	int ret = platform_device_register(&aic_device);

	if (ret) {
		pr_err("AIC: Failed to register aic dev: %d\n", ret);
		return ret;
	}
	return platform_driver_register(&aic_driver);
}

static void __exit aic_exit(void)
{
	platform_device_unregister(&aic_device);
	platform_driver_unregister(&aic_driver);
}

module_init(aic_init);
module_exit(aic_exit);

MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
