// SPDX-License-Identifier: GPL-2.0
//
// rmem_manager -- an allocator for physically contiguous memory, used by
// soc_fb.ko for the framebuffer: rmem_alloc(), rmem_alloc_aligned(),
// rmem_free(), and /dev/rmem_manager for user space.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock rmem_manager.sh passes them, read from rmem= on the
// kernel command line):
//
//   rmem_start   physical address of the reserved region
//   rmem_size    its size in bytes; 0: no region
//
// With a region, blocks come from it, first fit, with free neighbours merged
// back together. A request the region cannot satisfy, or any request without
// a region, goes to dma_alloc_coherent(). Addresses are KSEG0.
//
// /dev/rmem_manager ioctls, on struct { u32 size; u32 phys; }:
//   0xc0045278   allocate size bytes (page aligned), phys out
//   0xc0045279   free the block at phys
//   0xc004577a   on struct { u32 addr, size, to_cpu; }: dma_cache_sync()
// and mmap() of physical memory, uncached. Blocks a process still holds are
// freed when it closes the device.

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <asm/addrspace.h>

#define RMEM_IOC_ALLOC		0xc0045278
#define RMEM_IOC_FREE		0xc0045279
#define RMEM_IOC_CACHE_SYNC	0xc004577a

#define RMEM_POOL_BYTES		4096
#define RMEM_MIN_ALIGN		4
#define RMEM_MIN_SPLIT		4	// a smaller remainder stays with the block

// A block of the region: on free_list (sorted by start), on alloc_list, or
// spare on index_list. addr is the aligned address handed out.
struct rmem_block {
	struct list_head list;
	unsigned long start;
	unsigned long addr;
	int size;
};

// What one open file of /dev/rmem_manager holds.
struct rmem_file {
	struct list_head blocks;	// struct rmem_user
	struct mutex lock;
};

struct rmem_user {
	struct list_head list;
	unsigned int size;
	unsigned long addr;
};

static unsigned int rmem_size;
module_param(rmem_size, uint, 0644);
static unsigned long rmem_start;
module_param(rmem_start, ulong, 0644);

static struct device *rmem_src_dev;
static struct rmem_block *rmem_pool;

static DEFINE_MUTEX(rmem_lock);
static LIST_HEAD(index_list);
static LIST_HEAD(alloc_list);
static LIST_HEAD(free_list);

// --- the region -----------------------------------------------------------------

static void rmem_add_index(struct rmem_block *blk)
{
	blk->start = 0;
	blk->addr = 0;
	blk->size = 0;
	list_add_tail(&blk->list, &index_list);
}

// Puts a block back on the free list in address order, merged with the blocks
// it touches.
static void rmem_add_free(struct rmem_block *blk)
{
	struct rmem_block *pos, *prev;
	struct list_head *before;

	list_for_each_entry(pos, &free_list, list) {
		if (blk->start < pos->start) {
			before = pos->list.prev;
			if (blk->start + blk->size == pos->start) {
				list_del(&pos->list);
				blk->size += pos->size;
				rmem_add_index(pos);
			}
			prev = list_entry(before, struct rmem_block, list);
			if (before != &free_list && prev->start + prev->size == blk->start) {
				prev->size += blk->size;
				rmem_add_index(blk);
			} else {
				list_add(&blk->list, before);
			}
			return;
		}
		if (pos->list.next == &free_list && pos->start + pos->size == blk->start) {
			pos->size += blk->size;
			rmem_add_index(blk);
			return;
		}
	}
	list_add_tail(&blk->list, &free_list);
}

// size bytes aligned to align (a power of two, at least 4), or 0.
unsigned long rmem_alloc_aligned(int size, int align)
{
	struct rmem_block *blk, *rest;
	unsigned long addr = 0, aligned;
	int need;
	dma_addr_t handle;
	void *cpu;

	if (size <= 0)
		return 0;
	if (align < RMEM_MIN_ALIGN)
		align = RMEM_MIN_ALIGN;
	if (align & (align - 1))
		return 0;

	mutex_lock(&rmem_lock);
	list_for_each_entry(blk, &free_list, list) {
		aligned = (blk->start + align - 1) & -align;
		need = size + (aligned - blk->start);
		if (blk->size < need)
			continue;
		blk->addr = aligned;
		if (need == blk->size || blk->size - need < RMEM_MIN_SPLIT) {
			list_del(&blk->list);
		} else {
			if (list_empty(&index_list)) {
				printk(KERN_ERR "rmem: index memory is full\n");
				goto out;
			}
			rest = list_first_entry(&index_list, struct rmem_block, list);
			list_del(&rest->list);
			list_del(&blk->list);
			rest->start = blk->start + need;
			rest->size = blk->size - need;
			rmem_add_free(rest);
			blk->addr = aligned;
			blk->size = need;
		}
		list_add_tail(&blk->list, &alloc_list);
		addr = blk->addr;
		goto out;
	}

	cpu = dma_alloc_coherent(rmem_src_dev, size, &handle, GFP_KERNEL);
	if (cpu)
		addr = CKSEG0ADDR(cpu);
out:
	mutex_unlock(&rmem_lock);
	return addr;
}
EXPORT_SYMBOL(rmem_alloc_aligned);

unsigned long rmem_alloc(int size)
{
	return rmem_alloc_aligned(size, RMEM_MIN_ALIGN);
}
EXPORT_SYMBOL(rmem_alloc);

void rmem_free(unsigned long addr, int size)
{
	unsigned long start = CKSEG0ADDR(rmem_start);
	struct rmem_block *blk;

	if (addr < start || addr > start + rmem_size) {
		dma_free_coherent(rmem_src_dev, size, (void *)CKSEG1ADDR(addr), virt_to_phys((void *)addr));
		return;
	}

	mutex_lock(&rmem_lock);
	list_for_each_entry(blk, &alloc_list, list) {
		if (blk->addr == addr) {
			list_del(&blk->list);
			rmem_add_free(blk);
			goto out;
		}
	}
	printk(KERN_ERR "rmem: this addr: %p not rmem_alloc\n", (void *)addr);
out:
	mutex_unlock(&rmem_lock);
}
EXPORT_SYMBOL(rmem_free);

// The region as one free block; the rest of the pool is spare blocks.
static int rmem_init(void)
{
	int i, n = RMEM_POOL_BYTES / sizeof(struct rmem_block);

	if (!rmem_size) {
		printk(KERN_INFO "rmem:no remem set, use kmalloc\n");
		return 0;
	}
	rmem_pool = kmalloc(RMEM_POOL_BYTES, GFP_KERNEL);
	if (!rmem_pool)
		return -ENOMEM;
	rmem_pool[0].start = CKSEG0ADDR(rmem_start);
	rmem_pool[0].addr = rmem_pool[0].start;
	rmem_pool[0].size = rmem_size;
	list_add_tail(&rmem_pool[0].list, &free_list);
	for (i = 1; i < n; i++)
		rmem_add_index(&rmem_pool[i]);
	return 0;
}

static void rmem_exit(void)
{
	struct rmem_block *first;

	if (!rmem_pool)
		return;
	first = list_first_entry_or_null(&free_list, struct rmem_block, list);
	WARN(!first || first->size != rmem_size, "rmem: blocks still allocated\n");
	INIT_LIST_HEAD(&index_list);
	INIT_LIST_HEAD(&alloc_list);
	INIT_LIST_HEAD(&free_list);
	kfree(rmem_pool);
	rmem_pool = NULL;
}

// --- /dev/rmem_manager ------------------------------------------------------------

static int rmem_open(struct inode *inode, struct file *file)
{
	struct rmem_file *pdata = vmalloc(sizeof(*pdata));

	if (!pdata)
		return -ENOMEM;
	INIT_LIST_HEAD(&pdata->blocks);
	mutex_init(&pdata->lock);
	file->private_data = pdata;
	return 0;
}

static int rmem_release(struct inode *inode, struct file *file)
{
	struct rmem_file *pdata = file->private_data;
	struct rmem_user *ent, *next;

	mutex_lock(&pdata->lock);
	list_for_each_entry_safe(ent, next, &pdata->blocks, list) {
		printk(KERN_WARNING "RMEM: source not free, pid = %d, name = %s, mem = %p, size = %d\n",
		       current->pid, current->comm, (void *)ent->addr, ent->size);
		rmem_free(ent->addr, ent->size);
		list_del(&ent->list);
		vfree(ent);
	}
	mutex_unlock(&pdata->lock);
	vfree(pdata);
	return 0;
}

static long rmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rmem_file *pdata = file->private_data;
	u32 __user *uarg = (u32 __user *)arg;
	struct rmem_user *ent;
	unsigned long addr;
	u32 v[3];

	switch (cmd) {
	case RMEM_IOC_ALLOC:
		if (get_user(v[0], &uarg[0]))
			return -EFAULT;
		v[0] = PAGE_ALIGN(v[0]);
		addr = rmem_alloc_aligned(v[0], PAGE_SIZE);
		if (put_user(addr + 0x80000000, &uarg[1])) {
			if (addr)
				rmem_free(addr, v[0]);
			return -EFAULT;
		}
		if (!addr) {
			printk(KERN_ERR "RMEM: alloc memery %d err\n", v[0]);
			return -1;
		}
		ent = vmalloc(sizeof(*ent));
		if (!ent) {
			rmem_free(addr, v[0]);
			return -ENOMEM;
		}
		ent->size = v[0];
		ent->addr = addr;
		mutex_lock(&pdata->lock);
		list_add_tail(&ent->list, &pdata->blocks);
		mutex_unlock(&pdata->lock);
		return 0;
	case RMEM_IOC_FREE:
		if (copy_from_user(v, uarg, 2 * sizeof(u32)))
			return -EFAULT;
		addr = CKSEG0ADDR(v[1]);
		mutex_lock(&pdata->lock);
		list_for_each_entry(ent, &pdata->blocks, list) {
			if (ent->addr == addr) {
				list_del(&ent->list);
				vfree(ent);
				mutex_unlock(&pdata->lock);
				rmem_free(addr, PAGE_ALIGN(v[0]));
				return 0;
			}
		}
		printk(KERN_WARNING "RMEM: can not find this mem = %p\n", (void *)addr);
		mutex_unlock(&pdata->lock);
		return 0;
	case RMEM_IOC_CACHE_SYNC:
		if (copy_from_user(v, uarg, sizeof(v)))
			return -EFAULT;
		dma_cache_sync(NULL, (void *)v[0], v[1], v[2] ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		return 0;
	default:
		printk(KERN_ERR "RMEM: not support this cmd:%d\n", cmd);
		return -1;
	}
}

// Physical memory at the page offset, uncached.
static int rmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off + size - 1 < off)
		return -EINVAL;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = __pgprot(pgprot_val(vma->vm_page_prot) & ~_CACHE_MASK);
	vma->vm_pgoff = off >> PAGE_SHIFT;
	if (remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT, size, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static const struct file_operations rmem_misc_fops = {
	.unlocked_ioctl = rmem_ioctl,
	.mmap = rmem_mmap,
	.open = rmem_open,
	.release = rmem_release,
};

static struct miscdevice rmem_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rmem_manager",
	.fops = &rmem_misc_fops,
};

// --- platform devices -------------------------------------------------------------

static int rmem_manager_probe(struct platform_device *pdev)
{
	int ret;

	rmem_src_dev = &pdev->dev;
	ret = rmem_init();
	if (ret)
		return ret;
	ret = misc_register(&rmem_mdev);
	if (ret < 0)
		rmem_exit();
	return ret;
}

static int rmem_manager_remove(struct platform_device *pdev)
{
	rmem_exit();
	misc_deregister(&rmem_mdev);
	return 0;
}

static void jz_hash_dev_release(struct device *dev)
{
}

static struct platform_device rmem_manager_device = {
	.name = "rmem-manager",
	.id = 0,
	.dev = {
		.release = jz_hash_dev_release,
	},
};

static struct platform_driver rmem_manager_driver = {
	.probe = rmem_manager_probe,
	.remove = rmem_manager_remove,
	.driver = {
		.name = "rmem-manager",
		.owner = THIS_MODULE,
	},
};

// A device tree node with its own reserved memory becomes the device the
// fallback allocations are made for.
static int rmem_src_probe(struct platform_device *pdev)
{
	int ret = of_reserved_mem_device_init(&pdev->dev);

	if (ret) {
		dev_warn(&pdev->dev, "failed to init reserved mem\n");
		return ret;
	}
	rmem_src_dev = &pdev->dev;
	return 0;
}

static int rmem_src_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rmem_src_of_match[] = {
	{ .compatible = "ingenic,md-rmem-src" },
	{ }
};

static struct platform_driver rmem_src_driver = {
	.probe = rmem_src_probe,
	.remove = rmem_src_remove,
	.driver = {
		.name = "rmem_src",
		.of_match_table = rmem_src_of_match,
	},
};

static int __init rmem_manager_init(void)
{
	int ret;

	ret = platform_device_register(&rmem_manager_device);
	if (ret)
		return ret;
	ret = platform_driver_register(&rmem_manager_driver);
	if (ret)
		goto err_device;
	ret = platform_driver_register(&rmem_src_driver);
	if (ret)
		goto err_driver;
	return 0;

err_driver:
	platform_driver_unregister(&rmem_manager_driver);
err_device:
	platform_device_unregister(&rmem_manager_device);
	return ret;
}
module_init(rmem_manager_init);

static void __exit rmem_manager_exit(void)
{
	platform_driver_unregister(&rmem_src_driver);
	platform_device_unregister(&rmem_manager_device);
	platform_driver_unregister(&rmem_manager_driver);
}
module_exit(rmem_manager_exit);

MODULE_DESCRIPTION("Ingenic rmem_manager driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
