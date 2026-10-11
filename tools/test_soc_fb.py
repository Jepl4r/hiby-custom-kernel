#!/usr/bin/env python3
"""Host regression tests for the framebuffer mmap and pan paths.

The harness extracts the three functions under test from soc_fb.c and compiles
them against small host mocks. It exercises control flow, not real hardware;
unsigned-long arithmetic follows the host compiler's width. It needs only
Python 3 and a C compiler.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import tempfile


HARNESS = r'''
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

typedef uint32_t u32;
typedef unsigned long phys_addr_t;
struct mutex { int held; };
struct lcdc_desc { u32 next, addr, stride, cfg, ctrl; };
struct jzfb_mem { unsigned int xres, yres, frame_size, frame_num; };
struct jzfb_srdma { unsigned long mem; };
struct jzfb_par { int enabled; unsigned int mem_size; struct fb_info *fb; struct jzfb_mem m; struct jzfb_srdma srdma; };
struct fb_fix_screeninfo { unsigned long smem_start, smem_len; };
struct fb_info { void *par; struct fb_fix_screeninfo fix; };
struct fb_var_screeninfo { unsigned int xres, yres, xoffset, yoffset; };
struct vm_area_struct {
    unsigned long vm_start, vm_end, vm_pgoff, vm_flags;
    unsigned long vm_page_prot;
};
struct lcdc_data { unsigned int lcd_mode; };
struct jzfb {
    struct mutex lock;
    int irq_lock, wait_queue;
    struct lcdc_desc *desc[2];
    int frame_idx, state, pan_display_sync;
    struct lcdc_data *pdata;
};

#define PAGE_SHIFT 12
#define PAGE_SIZE (1UL << PAGE_SHIFT)
#define PAGE_MASK (~(PAGE_SIZE - 1))
#define PAGE_ALIGN(x) (((x) + PAGE_SIZE - 1) & PAGE_MASK)
#define VM_IO 0x4000UL
#define _CACHE_MASK 0x0fUL
#define _CACHE_CACHABLE_WT_WA 0x05UL
#define LCDC_FRM_DESC 0x1000
#define LCDC_FRM_START 0x1004
#define CPM_REG(off) (off)
#define LCDC_REG(off) (off)
#define SLCD_BUSY_PAN_US 10000
#define JZFB_WAIT_JIFFIES 30
#define LCD_MODE_IS_TFT(m) ((m) < 6)
#define JZFB_IDLE 0
#define JZFB_FETCHING 1
#define JZFB_FRAME_DONE 2
#define KERN_EMERG ""
#define KERN_ERR ""
#define printk(...) ((void)0)
#define READ_ONCE(x) (x)
#define pgprot_val(x) (x)
#define iob() ((void)0)
#define virt_to_phys(x) ((unsigned long)(uintptr_t)(x))

static struct jzfb jzfb;
static int remap_result, remap_calls, submitted, busy_result;
static unsigned long remap_pfn, remap_length;
static unsigned int wait_results[4];
static int wait_states[4], wait_count, wait_index, mutex_held_during_wait;
static unsigned long last_reg;
static struct jzfb_par *disable_on_lock;

static int remap_pfn_range(struct vm_area_struct *vma, unsigned long start,
                           unsigned long pfn, unsigned long size,
                           unsigned long prot) {
    (void)vma; (void)start; (void)prot;
    remap_calls++; remap_pfn=pfn; remap_length=size; return remap_result;
}
static void mutex_lock(struct mutex *m) {
    assert(!m->held); m->held=1;
    if (disable_on_lock) { disable_on_lock->enabled=0; disable_on_lock=NULL; }
}
static void mutex_unlock(struct mutex *m) { assert(m->held); m->held=0; }
#define spin_lock_irqsave(lock, flags) do { (void)(lock); (flags)=0; } while (0)
#define spin_unlock_irqrestore(lock, flags) do { (void)(lock); (void)(flags); } while (0)
static void writel(unsigned long value, unsigned long reg) {
    (void)value; last_reg=reg; if (reg == LCDC_FRM_START) submitted++;
}
static int wait_event_timeout(int queue, int condition, int timeout) {
    int i=wait_index++;
    (void)queue; (void)condition; (void)timeout;
    if (condition) return 1;
    assert(i < wait_count);
    if (jzfb.lock.held) mutex_held_during_wait++;
    jzfb.state=wait_states[i];
    return wait_results[i];
}
static int slcd_wait_busy_us(unsigned int usec) { (void)usec; return busy_result; }

__LCDC_PAN__
__JZFB_PAN__
__JZFB_MMAP__

static struct lcdc_desc descs[2];
static struct lcdc_data panel;
static struct jzfb_par par;
static struct fb_info info;

static void reset(void) {
    memset(&jzfb,0,sizeof jzfb); memset(descs,0,sizeof descs);
    memset(&par,0,sizeof par); memset(&info,0,sizeof info);
    memset(&panel,0,sizeof panel);
    jzfb.desc[0]=&descs[0]; jzfb.desc[1]=&descs[1]; jzfb.pdata=&panel;
    par.enabled=1; par.m.xres=320; par.m.yres=100; par.m.frame_size=4096; par.m.frame_num=3;
    par.srdma.mem=0x100000; info.par=&par; par.fb=&info;
    remap_result=remap_calls=submitted=busy_result=wait_count=wait_index=0;
    mutex_held_during_wait=0; last_reg=0;
    disable_on_lock=NULL;
}
'''


def extract_function(source: str, name: str) -> str:
    pattern = re.compile(r"\b(?:static\s+)?(?:int|void)\s+" + re.escape(name) + r"\s*\(")
    match = None
    for candidate in pattern.finditer(source):
        close = source.find(")", candidate.end())
        if close >= 0 and re.match(r"\s*\{", source[close + 1:]):
            match = candidate
            break
    if match is None:
        raise ValueError(f"could not find function {name}")
    start = match.start()
    brace = source.find("{", match.end())
    if brace < 0:
        raise ValueError(f"could not find body for {name}")
    depth = 0
    for pos in range(brace, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start:pos + 1]
    raise ValueError(f"unterminated body for {name}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parents[1] / "modules/soc_fb/soc_fb.c")
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    source = args.source.read_text()
    generated = HARNESS.replace("__LCDC_PAN__", extract_function(source, "lcdc_pan_display"))
    generated = generated.replace("__JZFB_PAN__", extract_function(source, "jzfb_pan_display"))
    generated = generated.replace("__JZFB_MMAP__", extract_function(source, "jzfb_mmap"))
    generated += r'''
static void test_mmap_boundaries(void) {
    struct vm_area_struct v={.vm_start=0x10000,.vm_end=0x11000,.vm_page_prot=0xaa};
    reset(); info.fix.smem_len=PAGE_SIZE*3-1; info.fix.smem_start=0x12345000;
    v.vm_pgoff=2;
    assert(jzfb_mmap(&info,&v)==0);
    assert(v.vm_pgoff==0x12347UL && remap_pfn==0x12347UL && remap_length==PAGE_SIZE);
    assert((v.vm_flags & VM_IO) && (v.vm_page_prot & _CACHE_MASK)==_CACHE_CACHABLE_WT_WA);
    assert((v.vm_page_prot & ~_CACHE_MASK)==0xa0);
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x13000};
    assert(jzfb_mmap(&info,&v)==0); /* entire three-page allocation */
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11000,.vm_pgoff=2};
    assert(jzfb_mmap(&info,&v)==0); /* last valid page */
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11001,.vm_pgoff=2};
    assert(jzfb_mmap(&info,&v)==-EINVAL);
    v=(struct vm_area_struct){.vm_start=0x11000,.vm_end=0x10000};
    assert(jzfb_mmap(&info,&v)==-EINVAL);
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x10000};
    assert(jzfb_mmap(&info,&v)==-EINVAL);
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11000,.vm_pgoff=ULONG_MAX};
    assert(jzfb_mmap(&info,&v)==-EINVAL);
    info.fix.smem_len=ULONG_MAX-(PAGE_SIZE-2);
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11000};
    assert(jzfb_mmap(&info,&v)==-EINVAL); /* PAGE_ALIGN addition wraps */
    info.fix.smem_len=PAGE_SIZE*3;
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11000,.vm_pgoff=1};
    info.fix.smem_start=ULONG_MAX & PAGE_MASK;
    assert(jzfb_mmap(&info,&v)==-EINVAL);
    info.fix.smem_len=0;
    v=(struct vm_area_struct){.vm_start=0x10000,.vm_end=0x11000};
    assert(jzfb_mmap(&info,&v)==-ENOMEM);
}

static void test_pan_rejects_bad_offsets_and_geometry(void) {
    reset(); panel.lcd_mode=6; struct fb_var_screeninfo v={.xres=1,.yres=0,.yoffset=100};
    assert(jzfb_pan_display(&v,&info)==0); /* uses trusted framebuffer yres */
    assert(submitted==1 && descs[1].addr==par.srdma.mem+par.m.frame_size);
    reset(); v=(struct fb_var_screeninfo){.xres=320,.yres=100};
    par.m.yres=0; assert(jzfb_pan_display(&v,&info)==-EINVAL);
    reset(); v=(struct fb_var_screeninfo){.xres=1,.yres=0,.yoffset=0,.xoffset=1};
    assert(jzfb_pan_display(&v,&info)==-EINVAL);
    reset(); v=(struct fb_var_screeninfo){.xres=320,.yres=100,.yoffset=1};
    assert(jzfb_pan_display(&v,&info)==-EINVAL);
    reset(); v=(struct fb_var_screeninfo){.xres=320,.yres=100,.yoffset=300};
    assert(jzfb_pan_display(&v,&info)==-EINVAL);
    assert(submitted==0 && jzfb.frame_idx==0);
    reset(); panel.lcd_mode=0; v=(struct fb_var_screeninfo){.xres=320,.yres=100,.yoffset=200};
    wait_count=1; wait_results[0]=1; wait_states[0]=JZFB_FRAME_DONE;
    assert(jzfb_pan_display(&v,&info)==0); /* final whole frame */
    assert(submitted==1 && descs[1].addr==par.srdma.mem+2*par.m.frame_size);
    reset(); panel.lcd_mode=6; busy_result=1;
    v=(struct fb_var_screeninfo){.xres=1,.yres=0,.yoffset=100};
    assert(jzfb_pan_display(&v,&info)==-ETIMEDOUT); /* pre-submit failure propagates */
    assert(submitted==0 && jzfb.frame_idx==0);
    reset(); par.enabled=0; v=(struct fb_var_screeninfo){.xres=320,.yres=100,.yoffset=200};
    assert(jzfb_pan_display(&v,&info)==-EBUSY); /* callback error propagates */
}

static void test_tft_timeout_and_success(void) {
    reset(); panel.lcd_mode=0; jzfb.pan_display_sync=1;
    wait_count=1; wait_results[0]=0; wait_states[0]=JZFB_FETCHING;
    assert(lcdc_pan_display(&par,1)==0); /* accepted after FRM_START despite wait timeout */
    assert(submitted==1 && mutex_held_during_wait==1 && !jzfb.lock.held);
    reset(); panel.lcd_mode=0; jzfb.pan_display_sync=1;
    wait_count=1; wait_results[0]=1; wait_states[0]=JZFB_FRAME_DONE;
    assert(lcdc_pan_display(&par,1)==0);
    assert(submitted==1 && mutex_held_during_wait==1 && descs[1].addr==0x101000);
}

static void test_smart_busy_and_wait_paths(void) {
    reset(); panel.lcd_mode=6; busy_result=1;
    struct lcdc_desc old0=descs[0], old1=descs[1];
    assert(lcdc_pan_display(&par,1)==-ETIMEDOUT);
    assert(submitted==0 && jzfb.frame_idx==0 && !jzfb.lock.held);
    assert(memcmp(&old0,&descs[0],sizeof old0)==0 && memcmp(&old1,&descs[1],sizeof old1)==0);
    reset(); panel.lcd_mode=6; jzfb.state=JZFB_FETCHING;
    old0=descs[0]; old1=descs[1];
    wait_count=1; wait_results[0]=0; wait_states[0]=JZFB_FETCHING;
    assert(lcdc_pan_display(&par,1)==-ETIMEDOUT);
    assert(submitted==0 && mutex_held_during_wait==1 && jzfb.frame_idx==0);
    assert(memcmp(&old0,&descs[0],sizeof old0)==0 && memcmp(&old1,&descs[1],sizeof old1)==0);
    reset(); panel.lcd_mode=6; jzfb.state=JZFB_FETCHING;
    wait_count=1; wait_results[0]=1; wait_states[0]=JZFB_FRAME_DONE; busy_result=1;
    assert(lcdc_pan_display(&par,1)==-ETIMEDOUT);
    assert(submitted==0 && mutex_held_during_wait==1);
    reset(); panel.lcd_mode=6; jzfb.pan_display_sync=1;
    wait_count=1; wait_results[0]=0; wait_states[0]=JZFB_FETCHING;
    assert(lcdc_pan_display(&par,1)==0); /* accepted after FRM_START despite sync timeout */
    assert(submitted==1 && mutex_held_during_wait==1 && !jzfb.lock.held);
    reset(); panel.lcd_mode=6; jzfb.pan_display_sync=1;
    wait_count=1; wait_results[0]=1; wait_states[0]=JZFB_FRAME_DONE;
    assert(lcdc_pan_display(&par,1)==0);
    assert(submitted==1 && mutex_held_during_wait==1);
    reset(); panel.lcd_mode=6; jzfb.pan_display_sync=0;
    assert(lcdc_pan_display(&par,1)==0); /* asynchronous smart-panel submission */
    assert(submitted==1 && wait_index==0 && !jzfb.lock.held);
}

static void test_disabled_is_side_effect_free(void) {
    reset(); par.enabled=0; panel.lcd_mode=0;
    struct lcdc_desc before0=descs[0], before1=descs[1];
    jzfb.lock.held=1; /* early enabled check must avoid recursively taking mutex */
    assert(lcdc_pan_display(&par,1)==-EBUSY);
    assert(jzfb.frame_idx==0 && submitted==0);
    assert(memcmp(&before0,&descs[0],sizeof before0)==0 && memcmp(&before1,&descs[1],sizeof before1)==0);
    assert(jzfb.lock.held==1);
    reset(); panel.lcd_mode=0; disable_on_lock=&par;
    before0=descs[0]; before1=descs[1];
    assert(lcdc_pan_display(&par,1)==-EBUSY); /* disable between fast check and lock */
    assert(jzfb.frame_idx==0 && submitted==0 && !jzfb.lock.held);
    assert(memcmp(&before0,&descs[0],sizeof before0)==0 && memcmp(&before1,&descs[1],sizeof before1)==0);
}

int main(void) {
    test_mmap_boundaries(); test_pan_rejects_bad_offsets_and_geometry();
    test_tft_timeout_and_success(); test_smart_busy_and_wait_paths();
    test_disabled_is_side_effect_free();
    puts("soc_fb host regressions: PASS"); return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix="soc-fb-host-test-") as tmp:
        c_path = pathlib.Path(tmp) / "test_soc_fb.c"
        exe_path = pathlib.Path(tmp) / "test_soc_fb"
        c_path.write_text(generated)
        subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", str(c_path), "-o", str(exe_path)], check=True)
        subprocess.run([str(exe_path)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
