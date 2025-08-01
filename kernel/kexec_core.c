// SPDX-License-Identifier: GPL-2.0-only
/*
 * kexec.c - kexec system call core code.
 * Copyright (C) 2002-2004 Eric Biederman  <ebiederm@xmission.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/btf.h>
#include <linux/capability.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/kexec.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/highmem.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/ioport.h>
#include <linux/hardirq.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/utsname.h>
#include <linux/numa.h>
#include <linux/suspend.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/panic_notifier.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/vmalloc.h>
#include <linux/swap.h>
#include <linux/syscore_ops.h>
#include <linux/compiler.h>
#include <linux/hugetlb.h>
#include <linux/objtool.h>
#include <linux/kmsg_dump.h>

#include <asm/page.h>
#include <asm/sections.h>

#include <crypto/hash.h>
#include "kexec_internal.h"

atomic_t __kexec_lock = ATOMIC_INIT(0);

/* Flag to indicate we are going to kexec a new kernel */
bool kexec_in_progress = false;

bool kexec_file_dbg_print;

/*
 * When kexec transitions to the new kernel there is a one-to-one
 * mapping between physical and virtual addresses.  On processors
 * where you can disable the MMU this is trivial, and easy.  For
 * others it is still a simple predictable page table to setup.
 *
 * In that environment kexec copies the new kernel to its final
 * resting place.  This means I can only support memory whose
 * physical address can fit in an unsigned long.  In particular
 * addresses where (pfn << PAGE_SHIFT) > ULONG_MAX cannot be handled.
 * If the assembly stub has more restrictive requirements
 * KEXEC_SOURCE_MEMORY_LIMIT and KEXEC_DEST_MEMORY_LIMIT can be
 * defined more restrictively in <asm/kexec.h>.
 *
 * The code for the transition from the current kernel to the
 * new kernel is placed in the control_code_buffer, whose size
 * is given by KEXEC_CONTROL_PAGE_SIZE.  In the best case only a single
 * page of memory is necessary, but some architectures require more.
 * Because this memory must be identity mapped in the transition from
 * virtual to physical addresses it must live in the range
 * 0 - TASK_SIZE, as only the user space mappings are arbitrarily
 * modifiable.
 *
 * The assembly stub in the control code buffer is passed a linked list
 * of descriptor pages detailing the source pages of the new kernel,
 * and the destination addresses of those source pages.  As this data
 * structure is not used in the context of the current OS, it must
 * be self-contained.
 *
 * The code has been made to work with highmem pages and will use a
 * destination page in its final resting place (if it happens
 * to allocate it).  The end product of this is that most of the
 * physical address space, and most of RAM can be used.
 *
 * Future directions include:
 *  - allocating a page table with the control code buffer identity
 *    mapped, to simplify machine_kexec and make kexec_on_panic more
 *    reliable.
 */

/*
 * KIMAGE_NO_DEST is an impossible destination address..., for
 * allocating pages whose destination address we do not care about.
 */
#define KIMAGE_NO_DEST (-1UL)
#define PAGE_COUNT(x) (((x) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct page *kimage_alloc_page(struct kimage *image,
				       gfp_t gfp_mask,
				       unsigned long dest);

int sanity_check_segment_list(struct kimage *image)
{
	int i;
	unsigned long nr_segments = image->nr_segments;
	unsigned long total_pages = 0;
	unsigned long nr_pages = totalram_pages();

	/*
	 * Verify we have good destination addresses.  The caller is
	 * responsible for making certain we don't attempt to load
	 * the new image into invalid or reserved areas of RAM.  This
	 * just verifies it is an address we can use.
	 *
	 * Since the kernel does everything in page size chunks ensure
	 * the destination addresses are page aligned.  Too many
	 * special cases crop of when we don't do this.  The most
	 * insidious is getting overlapping destination addresses
	 * simply because addresses are changed to page size
	 * granularity.
	 */
	for (i = 0; i < nr_segments; i++) {
		unsigned long mstart, mend;

		mstart = image->segment[i].mem;
		mend   = mstart + image->segment[i].memsz;
		if (mstart > mend)
			return -EADDRNOTAVAIL;
		if ((mstart & ~PAGE_MASK) || (mend & ~PAGE_MASK))
			return -EADDRNOTAVAIL;
		if (mend >= KEXEC_DESTINATION_MEMORY_LIMIT)
			return -EADDRNOTAVAIL;
	}

	/* Verify our destination addresses do not overlap.
	 * If we alloed overlapping destination addresses
	 * through very weird things can happen with no
	 * easy explanation as one segment stops on another.
	 */
	for (i = 0; i < nr_segments; i++) {
		unsigned long mstart, mend;
		unsigned long j;

		mstart = image->segment[i].mem;
		mend   = mstart + image->segment[i].memsz;
		for (j = 0; j < i; j++) {
			unsigned long pstart, pend;

			pstart = image->segment[j].mem;
			pend   = pstart + image->segment[j].memsz;
			/* Do the segments overlap ? */
			if ((mend > pstart) && (mstart < pend))
				return -EINVAL;
		}
	}

	/* Ensure our buffer sizes are strictly less than
	 * our memory sizes.  This should always be the case,
	 * and it is easier to check up front than to be surprised
	 * later on.
	 */
	for (i = 0; i < nr_segments; i++) {
		if (image->segment[i].bufsz > image->segment[i].memsz)
			return -EINVAL;
	}

	/*
	 * Verify that no more than half of memory will be consumed. If the
	 * request from userspace is too large, a large amount of time will be
	 * wasted allocating pages, which can cause a soft lockup.
	 */
	for (i = 0; i < nr_segments; i++) {
		if (PAGE_COUNT(image->segment[i].memsz) > nr_pages / 2)
			return -EINVAL;

		total_pages += PAGE_COUNT(image->segment[i].memsz);
	}

	if (total_pages > nr_pages / 2)
		return -EINVAL;

#ifdef CONFIG_CRASH_DUMP
	/*
	 * Verify we have good destination addresses.  Normally
	 * the caller is responsible for making certain we don't
	 * attempt to load the new image into invalid or reserved
	 * areas of RAM.  But crash kernels are preloaded into a
	 * reserved area of ram.  We must ensure the addresses
	 * are in the reserved area otherwise preloading the
	 * kernel could corrupt things.
	 */

	if (image->type == KEXEC_TYPE_CRASH) {
		for (i = 0; i < nr_segments; i++) {
			unsigned long mstart, mend;

			mstart = image->segment[i].mem;
			mend = mstart + image->segment[i].memsz - 1;
			/* Ensure we are within the crash kernel limits */
			if ((mstart < phys_to_boot_phys(crashk_res.start)) ||
			    (mend > phys_to_boot_phys(crashk_res.end)))
				return -EADDRNOTAVAIL;
		}
	}
#endif

	/*
	 * The destination addresses are searched from system RAM rather than
	 * being allocated from the buddy allocator, so they are not guaranteed
	 * to be accepted by the current kernel.  Accept the destination
	 * addresses before kexec swaps their content with the segments' source
	 * pages to avoid accessing memory before it is accepted.
	 */
	for (i = 0; i < nr_segments; i++)
		accept_memory(image->segment[i].mem, image->segment[i].memsz);

	return 0;
}

struct kimage *do_kimage_alloc_init(void)
{
	struct kimage *image;

	/* Allocate a controlling structure */
	image = kzalloc(sizeof(*image), GFP_KERNEL);
	if (!image)
		return NULL;

	image->head = 0;
	image->entry = &image->head;
	image->last_entry = &image->head;
	image->control_page = ~0; /* By default this does not apply */
	image->type = KEXEC_TYPE_DEFAULT;

	/* Initialize the list of control pages */
	INIT_LIST_HEAD(&image->control_pages);

	/* Initialize the list of destination pages */
	INIT_LIST_HEAD(&image->dest_pages);

	/* Initialize the list of unusable pages */
	INIT_LIST_HEAD(&image->unusable_pages);

#ifdef CONFIG_CRASH_HOTPLUG
	image->hp_action = KEXEC_CRASH_HP_NONE;
	image->elfcorehdr_index = -1;
	image->elfcorehdr_updated = false;
#endif

	return image;
}

int kimage_is_destination_range(struct kimage *image,
					unsigned long start,
					unsigned long end)
{
	unsigned long i;

	for (i = 0; i < image->nr_segments; i++) {
		unsigned long mstart, mend;

		mstart = image->segment[i].mem;
		mend = mstart + image->segment[i].memsz - 1;
		if ((end >= mstart) && (start <= mend))
			return 1;
	}

	return 0;
}

static struct page *kimage_alloc_pages(gfp_t gfp_mask, unsigned int order)
{
	struct page *pages;

	if (fatal_signal_pending(current))
		return NULL;
	pages = alloc_pages(gfp_mask & ~__GFP_ZERO, order);
	if (pages) {
		unsigned int count, i;

		pages->mapping = NULL;
		set_page_private(pages, order);
		count = 1 << order;
		for (i = 0; i < count; i++)
			SetPageReserved(pages + i);

		arch_kexec_post_alloc_pages(page_address(pages), count,
					    gfp_mask);

		if (gfp_mask & __GFP_ZERO)
			for (i = 0; i < count; i++)
				clear_highpage(pages + i);
	}

	return pages;
}

static void kimage_free_pages(struct page *page)
{
	unsigned int order, count, i;

	order = page_private(page);
	count = 1 << order;

	arch_kexec_pre_free_pages(page_address(page), count);

	for (i = 0; i < count; i++)
		ClearPageReserved(page + i);
	__free_pages(page, order);
}

void kimage_free_page_list(struct list_head *list)
{
	struct page *page, *next;

	list_for_each_entry_safe(page, next, list, lru) {
		list_del(&page->lru);
		kimage_free_pages(page);
	}
}

static struct page *kimage_alloc_normal_control_pages(struct kimage *image,
							unsigned int order)
{
	/* Control pages are special, they are the intermediaries
	 * that are needed while we copy the rest of the pages
	 * to their final resting place.  As such they must
	 * not conflict with either the destination addresses
	 * or memory the kernel is already using.
	 *
	 * The only case where we really need more than one of
	 * these are for architectures where we cannot disable
	 * the MMU and must instead generate an identity mapped
	 * page table for all of the memory.
	 *
	 * At worst this runs in O(N) of the image size.
	 */
	struct list_head extra_pages;
	struct page *pages;
	unsigned int count;

	count = 1 << order;
	INIT_LIST_HEAD(&extra_pages);

	/* Loop while I can allocate a page and the page allocated
	 * is a destination page.
	 */
	do {
		unsigned long pfn, epfn, addr, eaddr;

		pages = kimage_alloc_pages(KEXEC_CONTROL_MEMORY_GFP, order);
		if (!pages)
			break;
		pfn   = page_to_boot_pfn(pages);
		epfn  = pfn + count;
		addr  = pfn << PAGE_SHIFT;
		eaddr = (epfn << PAGE_SHIFT) - 1;
		if ((epfn >= (KEXEC_CONTROL_MEMORY_LIMIT >> PAGE_SHIFT)) ||
			      kimage_is_destination_range(image, addr, eaddr)) {
			list_add(&pages->lru, &extra_pages);
			pages = NULL;
		}
	} while (!pages);

	if (pages) {
		/* Remember the allocated page... */
		list_add(&pages->lru, &image->control_pages);

		/* Because the page is already in it's destination
		 * location we will never allocate another page at
		 * that address.  Therefore kimage_alloc_pages
		 * will not return it (again) and we don't need
		 * to give it an entry in image->segment[].
		 */
	}
	/* Deal with the destination pages I have inadvertently allocated.
	 *
	 * Ideally I would convert multi-page allocations into single
	 * page allocations, and add everything to image->dest_pages.
	 *
	 * For now it is simpler to just free the pages.
	 */
	kimage_free_page_list(&extra_pages);

	return pages;
}

#ifdef CONFIG_CRASH_DUMP
static struct page *kimage_alloc_crash_control_pages(struct kimage *image,
						      unsigned int order)
{
	/* Control pages are special, they are the intermediaries
	 * that are needed while we copy the rest of the pages
	 * to their final resting place.  As such they must
	 * not conflict with either the destination addresses
	 * or memory the kernel is already using.
	 *
	 * Control pages are also the only pags we must allocate
	 * when loading a crash kernel.  All of the other pages
	 * are specified by the segments and we just memcpy
	 * into them directly.
	 *
	 * The only case where we really need more than one of
	 * these are for architectures where we cannot disable
	 * the MMU and must instead generate an identity mapped
	 * page table for all of the memory.
	 *
	 * Given the low demand this implements a very simple
	 * allocator that finds the first hole of the appropriate
	 * size in the reserved memory region, and allocates all
	 * of the memory up to and including the hole.
	 */
	unsigned long hole_start, hole_end, size;
	struct page *pages;

	pages = NULL;
	size = (1 << order) << PAGE_SHIFT;
	hole_start = ALIGN(image->control_page, size);
	hole_end   = hole_start + size - 1;
	while (hole_end <= crashk_res.end) {
		unsigned long i;

		cond_resched();

		if (hole_end > KEXEC_CRASH_CONTROL_MEMORY_LIMIT)
			break;
		/* See if I overlap any of the segments */
		for (i = 0; i < image->nr_segments; i++) {
			unsigned long mstart, mend;

			mstart = image->segment[i].mem;
			mend   = mstart + image->segment[i].memsz - 1;
			if ((hole_end >= mstart) && (hole_start <= mend)) {
				/* Advance the hole to the end of the segment */
				hole_start = ALIGN(mend, size);
				hole_end   = hole_start + size - 1;
				break;
			}
		}
		/* If I don't overlap any segments I have found my hole! */
		if (i == image->nr_segments) {
			pages = pfn_to_page(hole_start >> PAGE_SHIFT);
			image->control_page = hole_end + 1;
			break;
		}
	}

	/* Ensure that these pages are decrypted if SME is enabled. */
	if (pages)
		arch_kexec_post_alloc_pages(page_address(pages), 1 << order, 0);

	return pages;
}
#endif


struct page *kimage_alloc_control_pages(struct kimage *image,
					 unsigned int order)
{
	struct page *pages = NULL;

	switch (image->type) {
	case KEXEC_TYPE_DEFAULT:
		pages = kimage_alloc_normal_control_pages(image, order);
		break;
#ifdef CONFIG_CRASH_DUMP
	case KEXEC_TYPE_CRASH:
		pages = kimage_alloc_crash_control_pages(image, order);
		break;
#endif
	}

	return pages;
}

static int kimage_add_entry(struct kimage *image, kimage_entry_t entry)
{
	if (*image->entry != 0)
		image->entry++;

	if (image->entry == image->last_entry) {
		kimage_entry_t *ind_page;
		struct page *page;

		page = kimage_alloc_page(image, GFP_KERNEL, KIMAGE_NO_DEST);
		if (!page)
			return -ENOMEM;

		ind_page = page_address(page);
		*image->entry = virt_to_boot_phys(ind_page) | IND_INDIRECTION;
		image->entry = ind_page;
		image->last_entry = ind_page +
				      ((PAGE_SIZE/sizeof(kimage_entry_t)) - 1);
	}
	*image->entry = entry;
	image->entry++;
	*image->entry = 0;

	return 0;
}

static int kimage_set_destination(struct kimage *image,
				   unsigned long destination)
{
	destination &= PAGE_MASK;

	return kimage_add_entry(image, destination | IND_DESTINATION);
}


static int kimage_add_page(struct kimage *image, unsigned long page)
{
	page &= PAGE_MASK;

	return kimage_add_entry(image, page | IND_SOURCE);
}


static void kimage_free_extra_pages(struct kimage *image)
{
	/* Walk through and free any extra destination pages I may have */
	kimage_free_page_list(&image->dest_pages);

	/* Walk through and free any unusable pages I have cached */
	kimage_free_page_list(&image->unusable_pages);

}

void kimage_terminate(struct kimage *image)
{
	if (*image->entry != 0)
		image->entry++;

	*image->entry = IND_DONE;
}

#define for_each_kimage_entry(image, ptr, entry) \
	for (ptr = &image->head; (entry = *ptr) && !(entry & IND_DONE); \
		ptr = (entry & IND_INDIRECTION) ? \
			boot_phys_to_virt((entry & PAGE_MASK)) : ptr + 1)

static void kimage_free_entry(kimage_entry_t entry)
{
	struct page *page;

	page = boot_pfn_to_page(entry >> PAGE_SHIFT);
	kimage_free_pages(page);
}

void kimage_free(struct kimage *image)
{
	kimage_entry_t *ptr, entry;
	kimage_entry_t ind = 0;

	if (!image)
		return;

#ifdef CONFIG_CRASH_DUMP
	if (image->vmcoreinfo_data_copy) {
		crash_update_vmcoreinfo_safecopy(NULL);
		vunmap(image->vmcoreinfo_data_copy);
	}
#endif

	kimage_free_extra_pages(image);
	for_each_kimage_entry(image, ptr, entry) {
		if (entry & IND_INDIRECTION) {
			/* Free the previous indirection page */
			if (ind & IND_INDIRECTION)
				kimage_free_entry(ind);
			/* Save this indirection page until we are
			 * done with it.
			 */
			ind = entry;
		} else if (entry & IND_SOURCE)
			kimage_free_entry(entry);
	}
	/* Free the final indirection page */
	if (ind & IND_INDIRECTION)
		kimage_free_entry(ind);

	/* Handle any machine specific cleanup */
	machine_kexec_cleanup(image);

	/* Free the kexec control pages... */
	kimage_free_page_list(&image->control_pages);

	/*
	 * Free up any temporary buffers allocated. This might hit if
	 * error occurred much later after buffer allocation.
	 */
	if (image->file_mode)
		kimage_file_post_load_cleanup(image);

	kfree(image);
}

static kimage_entry_t *kimage_dst_used(struct kimage *image,
					unsigned long page)
{
	kimage_entry_t *ptr, entry;
	unsigned long destination = 0;

	for_each_kimage_entry(image, ptr, entry) {
		if (entry & IND_DESTINATION)
			destination = entry & PAGE_MASK;
		else if (entry & IND_SOURCE) {
			if (page == destination)
				return ptr;
			destination += PAGE_SIZE;
		}
	}

	return NULL;
}

static struct page *kimage_alloc_page(struct kimage *image,
					gfp_t gfp_mask,
					unsigned long destination)
{
	/*
	 * Here we implement safeguards to ensure that a source page
	 * is not copied to its destination page before the data on
	 * the destination page is no longer useful.
	 *
	 * To do this we maintain the invariant that a source page is
	 * either its own destination page, or it is not a
	 * destination page at all.
	 *
	 * That is slightly stronger than required, but the proof
	 * that no problems will not occur is trivial, and the
	 * implementation is simply to verify.
	 *
	 * When allocating all pages normally this algorithm will run
	 * in O(N) time, but in the worst case it will run in O(N^2)
	 * time.   If the runtime is a problem the data structures can
	 * be fixed.
	 */
	struct page *page;
	unsigned long addr;

	/*
	 * Walk through the list of destination pages, and see if I
	 * have a match.
	 */
	list_for_each_entry(page, &image->dest_pages, lru) {
		addr = page_to_boot_pfn(page) << PAGE_SHIFT;
		if (addr == destination) {
			list_del(&page->lru);
			return page;
		}
	}
	page = NULL;
	while (1) {
		kimage_entry_t *old;

		/* Allocate a page, if we run out of memory give up */
		page = kimage_alloc_pages(gfp_mask, 0);
		if (!page)
			return NULL;
		/* If the page cannot be used file it away */
		if (page_to_boot_pfn(page) >
				(KEXEC_SOURCE_MEMORY_LIMIT >> PAGE_SHIFT)) {
			list_add(&page->lru, &image->unusable_pages);
			continue;
		}
		addr = page_to_boot_pfn(page) << PAGE_SHIFT;

		/* If it is the destination page we want use it */
		if (addr == destination)
			break;

		/* If the page is not a destination page use it */
		if (!kimage_is_destination_range(image, addr,
						  addr + PAGE_SIZE - 1))
			break;

		/*
		 * I know that the page is someones destination page.
		 * See if there is already a source page for this
		 * destination page.  And if so swap the source pages.
		 */
		old = kimage_dst_used(image, addr);
		if (old) {
			/* If so move it */
			unsigned long old_addr;
			struct page *old_page;

			old_addr = *old & PAGE_MASK;
			old_page = boot_pfn_to_page(old_addr >> PAGE_SHIFT);
			copy_highpage(page, old_page);
			*old = addr | (*old & ~PAGE_MASK);

			/* The old page I have found cannot be a
			 * destination page, so return it if it's
			 * gfp_flags honor the ones passed in.
			 */
			if (!(gfp_mask & __GFP_HIGHMEM) &&
			    PageHighMem(old_page)) {
				kimage_free_pages(old_page);
				continue;
			}
			page = old_page;
			break;
		}
		/* Place the page on the destination list, to be used later */
		list_add(&page->lru, &image->dest_pages);
	}

	return page;
}

static int kimage_load_normal_segment(struct kimage *image,
					 struct kexec_segment *segment)
{
	unsigned long maddr;
	size_t ubytes, mbytes;
	int result;
	unsigned char __user *buf = NULL;
	unsigned char *kbuf = NULL;

	if (image->file_mode)
		kbuf = segment->kbuf;
	else
		buf = segment->buf;
	ubytes = segment->bufsz;
	mbytes = segment->memsz;
	maddr = segment->mem;

	result = kimage_set_destination(image, maddr);
	if (result < 0)
		goto out;

	while (mbytes) {
		struct page *page;
		char *ptr;
		size_t uchunk, mchunk;

		page = kimage_alloc_page(image, GFP_HIGHUSER, maddr);
		if (!page) {
			result  = -ENOMEM;
			goto out;
		}
		result = kimage_add_page(image, page_to_boot_pfn(page)
								<< PAGE_SHIFT);
		if (result < 0)
			goto out;

		ptr = kmap_local_page(page);
		/* Start with a clear page */
		clear_page(ptr);
		ptr += maddr & ~PAGE_MASK;
		mchunk = min_t(size_t, mbytes,
				PAGE_SIZE - (maddr & ~PAGE_MASK));
		uchunk = min(ubytes, mchunk);

		if (uchunk) {
			/* For file based kexec, source pages are in kernel memory */
			if (image->file_mode)
				memcpy(ptr, kbuf, uchunk);
			else
				result = copy_from_user(ptr, buf, uchunk);
			ubytes -= uchunk;
			if (image->file_mode)
				kbuf += uchunk;
			else
				buf += uchunk;
		}
		kunmap_local(ptr);
		if (result) {
			result = -EFAULT;
			goto out;
		}
		maddr  += mchunk;
		mbytes -= mchunk;

		cond_resched();
	}
out:
	return result;
}

#ifdef CONFIG_CRASH_DUMP
static int kimage_load_crash_segment(struct kimage *image,
					struct kexec_segment *segment)
{
	/* For crash dumps kernels we simply copy the data from
	 * user space to it's destination.
	 * We do things a page at a time for the sake of kmap.
	 */
	unsigned long maddr;
	size_t ubytes, mbytes;
	int result;
	unsigned char __user *buf = NULL;
	unsigned char *kbuf = NULL;

	result = 0;
	if (image->file_mode)
		kbuf = segment->kbuf;
	else
		buf = segment->buf;
	ubytes = segment->bufsz;
	mbytes = segment->memsz;
	maddr = segment->mem;
	while (mbytes) {
		struct page *page;
		char *ptr;
		size_t uchunk, mchunk;

		page = boot_pfn_to_page(maddr >> PAGE_SHIFT);
		if (!page) {
			result  = -ENOMEM;
			goto out;
		}
		arch_kexec_post_alloc_pages(page_address(page), 1, 0);
		ptr = kmap_local_page(page);
		ptr += maddr & ~PAGE_MASK;
		mchunk = min_t(size_t, mbytes,
				PAGE_SIZE - (maddr & ~PAGE_MASK));
		uchunk = min(ubytes, mchunk);
		if (mchunk > uchunk) {
			/* Zero the trailing part of the page */
			memset(ptr + uchunk, 0, mchunk - uchunk);
		}

		if (uchunk) {
			/* For file based kexec, source pages are in kernel memory */
			if (image->file_mode)
				memcpy(ptr, kbuf, uchunk);
			else
				result = copy_from_user(ptr, buf, uchunk);
			ubytes -= uchunk;
			if (image->file_mode)
				kbuf += uchunk;
			else
				buf += uchunk;
		}
		kexec_flush_icache_page(page);
		kunmap_local(ptr);
		arch_kexec_pre_free_pages(page_address(page), 1);
		if (result) {
			result = -EFAULT;
			goto out;
		}
		maddr  += mchunk;
		mbytes -= mchunk;

		cond_resched();
	}
out:
	return result;
}
#endif

int kimage_load_segment(struct kimage *image,
				struct kexec_segment *segment)
{
	int result = -ENOMEM;

	switch (image->type) {
	case KEXEC_TYPE_DEFAULT:
		result = kimage_load_normal_segment(image, segment);
		break;
#ifdef CONFIG_CRASH_DUMP
	case KEXEC_TYPE_CRASH:
		result = kimage_load_crash_segment(image, segment);
		break;
#endif
	}

	return result;
}

void *kimage_map_segment(struct kimage *image,
			 unsigned long addr, unsigned long size)
{
	unsigned long src_page_addr, dest_page_addr = 0;
	unsigned long eaddr = addr + size;
	kimage_entry_t *ptr, entry;
	struct page **src_pages;
	unsigned int npages;
	void *vaddr = NULL;
	int i;

	/*
	 * Collect the source pages and map them in a contiguous VA range.
	 */
	npages = PFN_UP(eaddr) - PFN_DOWN(addr);
	src_pages = kmalloc_array(npages, sizeof(*src_pages), GFP_KERNEL);
	if (!src_pages) {
		pr_err("Could not allocate ima pages array.\n");
		return NULL;
	}

	i = 0;
	for_each_kimage_entry(image, ptr, entry) {
		if (entry & IND_DESTINATION) {
			dest_page_addr = entry & PAGE_MASK;
		} else if (entry & IND_SOURCE) {
			if (dest_page_addr >= addr && dest_page_addr < eaddr) {
				src_page_addr = entry & PAGE_MASK;
				src_pages[i++] =
					virt_to_page(__va(src_page_addr));
				if (i == npages)
					break;
				dest_page_addr += PAGE_SIZE;
			}
		}
	}

	/* Sanity check. */
	WARN_ON(i < npages);

	vaddr = vmap(src_pages, npages, VM_MAP, PAGE_KERNEL);
	kfree(src_pages);

	if (!vaddr)
		pr_err("Could not map ima buffer.\n");

	return vaddr;
}

void kimage_unmap_segment(void *segment_buffer)
{
	vunmap(segment_buffer);
}

struct kexec_load_limit {
	/* Mutex protects the limit count. */
	struct mutex mutex;
	int limit;
};

static struct kexec_load_limit load_limit_reboot = {
	.mutex = __MUTEX_INITIALIZER(load_limit_reboot.mutex),
	.limit = -1,
};

static struct kexec_load_limit load_limit_panic = {
	.mutex = __MUTEX_INITIALIZER(load_limit_panic.mutex),
	.limit = -1,
};

struct kimage *kexec_image;
struct kimage *kexec_crash_image;
static int kexec_load_disabled;

#ifdef CONFIG_SYSCTL
static int kexec_limit_handler(const struct ctl_table *table, int write,
			       void *buffer, size_t *lenp, loff_t *ppos)
{
	struct kexec_load_limit *limit = table->data;
	int val;
	struct ctl_table tmp = {
		.data = &val,
		.maxlen = sizeof(val),
		.mode = table->mode,
	};
	int ret;

	if (write) {
		ret = proc_dointvec(&tmp, write, buffer, lenp, ppos);
		if (ret)
			return ret;

		if (val < 0)
			return -EINVAL;

		mutex_lock(&limit->mutex);
		if (limit->limit != -1 && val >= limit->limit)
			ret = -EINVAL;
		else
			limit->limit = val;
		mutex_unlock(&limit->mutex);

		return ret;
	}

	mutex_lock(&limit->mutex);
	val = limit->limit;
	mutex_unlock(&limit->mutex);

	return proc_dointvec(&tmp, write, buffer, lenp, ppos);
}

static const struct ctl_table kexec_core_sysctls[] = {
	{
		.procname	= "kexec_load_disabled",
		.data		= &kexec_load_disabled,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		/* only handle a transition from default "0" to "1" */
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= SYSCTL_ONE,
		.extra2		= SYSCTL_ONE,
	},
	{
		.procname	= "kexec_load_limit_panic",
		.data		= &load_limit_panic,
		.mode		= 0644,
		.proc_handler	= kexec_limit_handler,
	},
	{
		.procname	= "kexec_load_limit_reboot",
		.data		= &load_limit_reboot,
		.mode		= 0644,
		.proc_handler	= kexec_limit_handler,
	},
};

static int __init kexec_core_sysctl_init(void)
{
	register_sysctl_init("kernel", kexec_core_sysctls);
	return 0;
}
late_initcall(kexec_core_sysctl_init);
#endif

bool kexec_load_permitted(int kexec_image_type)
{
	struct kexec_load_limit *limit;

	/*
	 * Only the superuser can use the kexec syscall and if it has not
	 * been disabled.
	 */
	if (!capable(CAP_SYS_BOOT) || kexec_load_disabled)
		return false;

	/* Check limit counter and decrease it.*/
	limit = (kexec_image_type == KEXEC_TYPE_CRASH) ?
		&load_limit_panic : &load_limit_reboot;
	mutex_lock(&limit->mutex);
	if (!limit->limit) {
		mutex_unlock(&limit->mutex);
		return false;
	}
	if (limit->limit != -1)
		limit->limit--;
	mutex_unlock(&limit->mutex);

	return true;
}

/*
 * Move into place and start executing a preloaded standalone
 * executable.  If nothing was preloaded return an error.
 */
int kernel_kexec(void)
{
	int error = 0;

	if (!kexec_trylock())
		return -EBUSY;
	if (!kexec_image) {
		error = -EINVAL;
		goto Unlock;
	}

#ifdef CONFIG_KEXEC_JUMP
	if (kexec_image->preserve_context) {
		/*
		 * This flow is analogous to hibernation flows that occur
		 * before creating an image and before jumping from the
		 * restore kernel to the image one, so it uses the same
		 * device callbacks as those two flows.
		 */
		pm_prepare_console();
		error = freeze_processes();
		if (error) {
			error = -EBUSY;
			goto Restore_console;
		}
		console_suspend_all();
		error = dpm_suspend_start(PMSG_FREEZE);
		if (error)
			goto Resume_devices;
		/*
		 * dpm_suspend_end() must be called after dpm_suspend_start()
		 * to complete the transition, like in the hibernation flows
		 * mentioned above.
		 */
		error = dpm_suspend_end(PMSG_FREEZE);
		if (error)
			goto Resume_devices;
		error = suspend_disable_secondary_cpus();
		if (error)
			goto Enable_cpus;
		local_irq_disable();
		error = syscore_suspend();
		if (error)
			goto Enable_irqs;
	} else
#endif
	{
		kexec_in_progress = true;
		kernel_restart_prepare("kexec reboot");
		migrate_to_reboot_cpu();
		syscore_shutdown();

		/*
		 * migrate_to_reboot_cpu() disables CPU hotplug assuming that
		 * no further code needs to use CPU hotplug (which is true in
		 * the reboot case). However, the kexec path depends on using
		 * CPU hotplug again; so re-enable it here.
		 */
		cpu_hotplug_enable();
		pr_notice("Starting new kernel\n");
		machine_shutdown();
	}

	kmsg_dump(KMSG_DUMP_SHUTDOWN);
	machine_kexec(kexec_image);

#ifdef CONFIG_KEXEC_JUMP
	if (kexec_image->preserve_context) {
		/*
		 * This flow is analogous to hibernation flows that occur after
		 * creating an image and after the image kernel has got control
		 * back, and in case the devices have been reset or otherwise
		 * manipulated in the meantime, it uses the device callbacks
		 * used by the latter.
		 */
		syscore_resume();
 Enable_irqs:
		local_irq_enable();
 Enable_cpus:
		suspend_enable_secondary_cpus();
		dpm_resume_start(PMSG_RESTORE);
 Resume_devices:
		dpm_resume_end(PMSG_RESTORE);
		console_resume_all();
		thaw_processes();
 Restore_console:
		pm_restore_console();
	}
#endif

 Unlock:
	kexec_unlock();
	return error;
}
