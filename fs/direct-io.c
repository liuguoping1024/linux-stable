// SPDX-License-Identifier: GPL-2.0-only
/*
 * fs/direct-io.c
 *
 * Copyright (C) 2002, Linus Torvalds.
 *
 * O_DIRECT
 *
 * 04Jul2002	Andrew Morton
 *		Initial version
 * 11Sep2002	janetinc@us.ibm.com
 * 		added readv/writev support.
 * 29Oct2002	Andrew Morton
 *		rewrote bio_add_page() support.
 * 30Oct2002	pbadari@us.ibm.com
 *		added support for non-aligned IO.
 * 06Nov2002	pbadari@us.ibm.com
 *		added asynchronous IO support.
 * 21Jul2003	nathans@sgi.com
 *		added IO completion notifier.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/bio.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/rwsem.h>
#include <linux/uio.h>
#include <linux/atomic.h>

#include "internal.h"

/*
 * How many user pages to map in one call to iov_iter_extract_pages().  This
 * determines the size of a structure in the slab cache
 */
#define DIO_PAGES	64

/*
 * Flags for dio_complete()
 */
#define DIO_COMPLETE_ASYNC		0x01	/* This is async IO */
#define DIO_COMPLETE_INVALIDATE		0x02	/* Can invalidate pages */

/*
 * This code generally works in units of "dio_blocks".  A dio_block is
 * somewhere between the hard sector size and the filesystem block size.  it
 * is determined on a per-invocation basis.   When talking to the filesystem
 * we need to convert dio_blocks to fs_blocks by scaling the dio_block quantity
 * down by dio->blkfactor.  Similarly, fs-blocksize quantities are converted
 * to bio_block quantities by shifting left by blkfactor.
 *
 * If blkfactor is zero then the user's request was aligned to the filesystem's
 * blocksize.
 */

/* dio_state only used in the submission path */

struct dio_submit {
	struct bio *bio;		/* bio under assembly */
	unsigned blkbits;		/* doesn't change */
	unsigned blkfactor;		/* When we're using an alignment which
					   is finer than the filesystem's soft
					   blocksize, this specifies how much
					   finer.  blkfactor=2 means 1/4-block
					   alignment.  Does not change */
	unsigned start_zero_done;	/* flag: sub-blocksize zeroing has
					   been performed at the start of a
					   write */
	int pages_in_io;		/* approximate total IO pages */
	sector_t block_in_file;		/* Current offset into the underlying
					   file in dio_block units. */
	unsigned blocks_available;	/* At block_in_file.  changes */
	int reap_counter;		/* rate limit reaping */
	sector_t final_block_in_request;/* doesn't change */
	int boundary;			/* prev block is at a boundary */
	get_block_t *get_block;		/* block mapping function */

	loff_t logical_offset_in_bio;	/* current first logical block in bio */
	sector_t final_block_in_bio;	/* current final block in bio + 1 */
	sector_t next_block_for_io;	/* next block to be put under IO,
					   in dio_blocks units */

	/*
	 * Deferred addition of a page to the dio.  These variables are
	 * private to dio_send_cur_page(), submit_page_section() and
	 * dio_bio_add_page().
	 */
	struct page *cur_page;		/* The page */
	unsigned cur_page_offset;	/* Offset into it, in bytes */
	unsigned cur_page_len;		/* Nr of bytes at cur_page_offset */
	sector_t cur_page_block;	/* Where it starts */
	loff_t cur_page_fs_offset;	/* Offset in file */

	struct iov_iter *iter;
	/*
	 * Page queue.  These variables belong to dio_refill_pages() and
	 * dio_get_page().
	 */
	unsigned head;			/* next page to process */
	unsigned tail;			/* last valid page + 1 */
	size_t from, to;
};

/* dio_state communicated between submission path and end_io */
struct dio {
	int flags;			/* doesn't change */
	blk_opf_t opf;			/* request operation type and flags */
	struct gendisk *bio_disk;
	struct inode *inode;
	loff_t i_size;			/* i_size when submitted */
	dio_iodone_t *end_io;		/* IO completion function */
	bool is_pinned;			/* T if we have pins on the pages */

	void *private;			/* copy from map_bh.b_private */

	/* BIO completion state */
	spinlock_t bio_lock;		/* protects BIO fields below */
	int page_errors;		/* err from iov_iter_extract_pages() */
	int is_async;			/* is IO async ? */
	bool defer_completion;		/* defer AIO completion to workqueue? */
	bool should_dirty;		/* if pages should be dirtied */
	int io_error;			/* IO error in completion path */
	unsigned long refcount;		/* direct_io_worker() and bios */
	struct bio *bio_list;		/* singly linked via bi_private */
	struct task_struct *waiter;	/* waiting task (NULL if none) */

	/* AIO related stuff */
	struct kiocb *iocb;		/* kiocb */
	ssize_t result;                 /* IO result */

	/*
	 * pages[] (and any fields placed after it) are not zeroed out at
	 * allocation time.  Don't add new fields after pages[] unless you
	 * wish that they not be zeroed.
	 */
	union {
		struct page *pages[DIO_PAGES];	/* page buffer */
		struct work_struct complete_work;/* deferred AIO completion */
	};
} ____cacheline_aligned_in_smp;

static struct kmem_cache *dio_cache __ro_after_init;

/*
 * How many pages are in the queue?
 */
static inline unsigned dio_pages_present(struct dio_submit *sdio)
{
	return sdio->tail - sdio->head;
}

/*
 * Go grab and pin some userspace pages.   Typically we'll get 64 at a time.
 */
static inline int dio_refill_pages(struct dio *dio, struct dio_submit *sdio)
{
	struct page **pages = dio->pages;
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	ssize_t ret;

	ret = iov_iter_extract_pages(sdio->iter, &pages, LONG_MAX,
				     DIO_PAGES, 0, &sdio->from);

	if (ret < 0 && sdio->blocks_available && dio_op == REQ_OP_WRITE) {
		/*
		 * A memory fault, but the filesystem has some outstanding
		 * mapped blocks.  We need to use those blocks up to avoid
		 * leaking stale data in the file.
		 */
		if (dio->page_errors == 0)
			dio->page_errors = ret;
		dio->pages[0] = ZERO_PAGE(0);
		sdio->head = 0;
		sdio->tail = 1;
		sdio->from = 0;
		sdio->to = PAGE_SIZE;
		return 0;
	}

	if (ret >= 0) {
		ret += sdio->from;
		sdio->head = 0;
		sdio->tail = (ret + PAGE_SIZE - 1) / PAGE_SIZE;
		sdio->to = ((ret - 1) & (PAGE_SIZE - 1)) + 1;
		return 0;
	}
	return ret;	
}

/*
 * Get another userspace page.  Returns an ERR_PTR on error.  Pages are
 * buffered inside the dio so that we can call iov_iter_extract_pages()
 * against a decent number of pages, less frequently.  To provide nicer use of
 * the L1 cache.
 */
static inline struct page *dio_get_page(struct dio *dio,
					struct dio_submit *sdio)
{
	if (dio_pages_present(sdio) == 0) {
		int ret;

		ret = dio_refill_pages(dio, sdio);
		if (ret)
			return ERR_PTR(ret);
		BUG_ON(dio_pages_present(sdio) == 0);
	}
	return dio->pages[sdio->head];
}

static void dio_pin_page(struct dio *dio, struct page *page)
{
	if (dio->is_pinned)
		folio_add_pin(page_folio(page));
}

static void dio_unpin_page(struct dio *dio, struct page *page)
{
	if (dio->is_pinned)
		unpin_user_page(page);
}

/*
 * dio_complete() - called when all DIO BIO I/O has been completed
 *
 * This drops i_dio_count, lets interested parties know that a DIO operation
 * has completed, and calculates the resulting return code for the operation.
 *
 * It lets the filesystem know if it registered an interest earlier via
 * get_block.  Pass the private field of the map buffer_head so that
 * filesystems can use it to hold additional state between get_block calls and
 * dio_complete.
 */
static ssize_t dio_complete(struct dio *dio, ssize_t ret, unsigned int flags)
{
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	loff_t offset = dio->iocb->ki_pos;
	ssize_t transferred = 0;
	int err;

	/*
	 * AIO submission can race with bio completion to get here while
	 * expecting to have the last io completed by bio completion.
	 * In that case -EIOCBQUEUED is in fact not an error we want
	 * to preserve through this call.
	 */
	if (ret == -EIOCBQUEUED)
		ret = 0;

	if (dio->result) {
		transferred = dio->result;

		/* Check for short read case */
		if (dio_op == REQ_OP_READ &&
		    ((offset + transferred) > dio->i_size))
			transferred = dio->i_size - offset;
		/* ignore EFAULT if some IO has been done */
		if (unlikely(ret == -EFAULT) && transferred)
			ret = 0;
	}

	if (ret == 0)
		ret = dio->page_errors;
	if (ret == 0)
		ret = dio->io_error;
	if (ret == 0)
		ret = transferred;

	if (dio->end_io) {
		// XXX: ki_pos??
		err = dio->end_io(dio->iocb, offset, ret, dio->private);
		if (err)
			ret = err;
	}

	/*
	 * Try again to invalidate clean pages which might have been cached by
	 * non-direct readahead, or faulted in by get_user_pages() if the source
	 * of the write was an mmap'ed region of the file we're writing.  Either
	 * one is a pretty crazy thing to do, so we don't support it 100%.  If
	 * this invalidation fails, tough, the write still worked...
	 *
	 * And this page cache invalidation has to be after dio->end_io(), as
	 * some filesystems convert unwritten extents to real allocations in
	 * end_io() when necessary, otherwise a racing buffer read would cache
	 * zeros from unwritten extents.
	 */
	if (flags & DIO_COMPLETE_INVALIDATE &&
	    ret > 0 && dio_op == REQ_OP_WRITE)
		kiocb_invalidate_post_direct_write(dio->iocb, ret);

	inode_dio_end(dio->inode);

	if (flags & DIO_COMPLETE_ASYNC) {
		/*
		 * generic_write_sync expects ki_pos to have been updated
		 * already, but the submission path only does this for
		 * synchronous I/O.
		 */
		dio->iocb->ki_pos += transferred;

		if (ret > 0 && dio_op == REQ_OP_WRITE)
			ret = generic_write_sync(dio->iocb, ret);
		dio->iocb->ki_complete(dio->iocb, ret);
	}

	kmem_cache_free(dio_cache, dio);
	return ret;
}

static void dio_aio_complete_work(struct work_struct *work)
{
	struct dio *dio = container_of(work, struct dio, complete_work);

	dio_complete(dio, 0, DIO_COMPLETE_ASYNC | DIO_COMPLETE_INVALIDATE);
}

static blk_status_t dio_bio_complete(struct dio *dio, struct bio *bio);

/*
 * Asynchronous IO callback. 
 */
static void dio_bio_end_aio(struct bio *bio)
{
	struct dio *dio = bio->bi_private;
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	unsigned long remaining;
	unsigned long flags;
	bool defer_completion = false;

	/* cleanup the bio */
	dio_bio_complete(dio, bio);

	spin_lock_irqsave(&dio->bio_lock, flags);
	remaining = --dio->refcount;
	if (remaining == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (remaining == 0) {
		/*
		 * Defer completion when defer_completion is set or
		 * when the inode has pages mapped and this is AIO write.
		 * We need to invalidate those pages because there is a
		 * chance they contain stale data in the case buffered IO
		 * went in between AIO submission and completion into the
		 * same region.
		 */
		if (dio->result)
			defer_completion = dio->defer_completion ||
					   (dio_op == REQ_OP_WRITE &&
					    dio->inode->i_mapping->nrpages);
		if (defer_completion) {
			INIT_WORK(&dio->complete_work, dio_aio_complete_work);
			queue_work(dio->inode->i_sb->s_dio_done_wq,
				   &dio->complete_work);
		} else {
			dio_complete(dio, 0, DIO_COMPLETE_ASYNC);
		}
	}
}

/*
 * The BIO completion handler simply queues the BIO up for the process-context
 * handler.
 *
 * During I/O bi_private points at the dio.  After I/O, bi_private is used to
 * implement a singly-linked list of completed BIOs, at dio->bio_list.
 */
static void dio_bio_end_io(struct bio *bio)
{
	struct dio *dio = bio->bi_private;
	unsigned long flags;

	spin_lock_irqsave(&dio->bio_lock, flags);
	bio->bi_private = dio->bio_list;
	dio->bio_list = bio;
	if (--dio->refcount == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);
}

static inline void
dio_bio_alloc(struct dio *dio, struct dio_submit *sdio,
	      struct block_device *bdev,
	      sector_t first_sector, int nr_vecs)
{
	struct bio *bio;

	/*
	 * bio_alloc() is guaranteed to return a bio when allowed to sleep and
	 * we request a valid number of vectors.
	 */
	bio = bio_alloc(bdev, nr_vecs, dio->opf, GFP_KERNEL);
	bio->bi_iter.bi_sector = first_sector;
	if (dio->is_async)
		bio->bi_end_io = dio_bio_end_aio;
	else
		bio->bi_end_io = dio_bio_end_io;
	if (dio->is_pinned)
		bio_set_flag(bio, BIO_PAGE_PINNED);
	bio->bi_write_hint = file_inode(dio->iocb->ki_filp)->i_write_hint;

	sdio->bio = bio;
	sdio->logical_offset_in_bio = sdio->cur_page_fs_offset;
}

/*
 * In the AIO read case we speculatively dirty the pages before starting IO.
 * During IO completion, any of these pages which happen to have been written
 * back will be redirtied by bio_check_pages_dirty().
 *
 * bios hold a dio reference between submit_bio and ->end_io.
 */
static inline void dio_bio_submit(struct dio *dio, struct dio_submit *sdio)
{
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	struct bio *bio = sdio->bio;
	unsigned long flags;

	bio->bi_private = dio;

	spin_lock_irqsave(&dio->bio_lock, flags);
	dio->refcount++;
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (dio->is_async && dio_op == REQ_OP_READ && dio->should_dirty)
		bio_set_pages_dirty(bio);

	dio->bio_disk = bio->bi_bdev->bd_disk;

	submit_bio(bio);

	sdio->bio = NULL;
	sdio->boundary = 0;
	sdio->logical_offset_in_bio = 0;
}

/*
 * Release any resources in case of a failure
 */
static inline void dio_cleanup(struct dio *dio, struct dio_submit *sdio)
{
	if (dio->is_pinned)
		unpin_user_pages(dio->pages + sdio->head,
				 sdio->tail - sdio->head);
	sdio->head = sdio->tail;
}

/*
 * Wait for the next BIO to complete.  Remove it and return it.  NULL is
 * returned once all BIOs have been completed.  This must only be called once
 * all bios have been issued so that dio->refcount can only decrease.  This
 * requires that the caller hold a reference on the dio.
 */
static struct bio *dio_await_one(struct dio *dio)
{
	unsigned long flags;
	struct bio *bio = NULL;

	spin_lock_irqsave(&dio->bio_lock, flags);

	/*
	 * Wait as long as the list is empty and there are bios in flight.  bio
	 * completion drops the count, maybe adds to the list, and wakes while
	 * holding the bio_lock so we don't need set_current_state()'s barrier
	 * and can call it after testing our condition.
	 */
	while (dio->refcount > 1 && dio->bio_list == NULL) {
		__set_current_state(TASK_UNINTERRUPTIBLE);
		dio->waiter = current;
		spin_unlock_irqrestore(&dio->bio_lock, flags);
		blk_io_schedule();
		/* wake up sets us TASK_RUNNING */
		spin_lock_irqsave(&dio->bio_lock, flags);
		dio->waiter = NULL;
	}
	if (dio->bio_list) {
		bio = dio->bio_list;
		dio->bio_list = bio->bi_private;
	}
	spin_unlock_irqrestore(&dio->bio_lock, flags);
	return bio;
}

/*
 * Process one completed BIO.  No locks are held.
 */
static blk_status_t dio_bio_complete(struct dio *dio, struct bio *bio)
{
	blk_status_t err = bio->bi_status;
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	bool should_dirty = dio_op == REQ_OP_READ && dio->should_dirty;

	if (err) {
		if (err == BLK_STS_AGAIN && (bio->bi_opf & REQ_NOWAIT))
			dio->io_error = -EAGAIN;
		else
			dio->io_error = -EIO;
	}

	if (dio->is_async && should_dirty) {
		bio_check_pages_dirty(bio);	/* transfers ownership */
	} else {
		bio_release_pages(bio, should_dirty);
		bio_put(bio);
	}
	return err;
}

/*
 * Wait on and process all in-flight BIOs.  This must only be called once
 * all bios have been issued so that the refcount can only decrease.
 * This just waits for all bios to make it through dio_bio_complete.  IO
 * errors are propagated through dio->io_error and should be propagated via
 * dio_complete().
 */
static void dio_await_completion(struct dio *dio)
{
	struct bio *bio;
	do {
		bio = dio_await_one(dio);
		if (bio)
			dio_bio_complete(dio, bio);
	} while (bio);
}

/*
 * A really large O_DIRECT read or write can generate a lot of BIOs.  So
 * to keep the memory consumption sane we periodically reap any completed BIOs
 * during the BIO generation phase.
 *
 * This also helps to limit the peak amount of pinned userspace memory.
 */
static inline int dio_bio_reap(struct dio *dio, struct dio_submit *sdio)
{
	int ret = 0;

	if (sdio->reap_counter++ >= 64) {
		while (dio->bio_list) {
			unsigned long flags;
			struct bio *bio;
			int ret2;

			spin_lock_irqsave(&dio->bio_lock, flags);
			bio = dio->bio_list;
			dio->bio_list = bio->bi_private;
			spin_unlock_irqrestore(&dio->bio_lock, flags);
			ret2 = blk_status_to_errno(dio_bio_complete(dio, bio));
			if (ret == 0)
				ret = ret2;
		}
		sdio->reap_counter = 0;
	}
	return ret;
}

static int dio_set_defer_completion(struct dio *dio)
{
	struct super_block *sb = dio->inode->i_sb;

	if (dio->defer_completion)
		return 0;
	dio->defer_completion = true;
	if (!sb->s_dio_done_wq)
		return sb_init_dio_done_wq(sb);
	return 0;
}

/*
 * Call into the fs to map some more disk blocks.  We record the current number
 * of available blocks at sdio->blocks_available.  These are in units of the
 * fs blocksize, i_blocksize(inode).
 *
 * The fs is allowed to map lots of blocks at once.  If it wants to do that,
 * it uses the passed inode-relative block number as the file offset, as usual.
 *
 * get_block() is passed the number of i_blkbits-sized blocks which direct_io
 * has remaining to do.  The fs should not map more than this number of blocks.
 *
 * If the fs has mapped a lot of blocks, it should populate bh->b_size to
 * indicate how much contiguous disk space has been made available at
 * bh->b_blocknr.
 *
 * If *any* of the mapped blocks are new, then the fs must set buffer_new().
 * This isn't very efficient...
 *
 * In the case of filesystem holes: the fs may return an arbitrarily-large
 * hole by returning an appropriate value in b_size and by clearing
 * buffer_mapped().  However the direct-io code will only process holes one
 * block at a time - it will repeatedly call get_block() as it walks the hole.
 */
static int get_more_blocks(struct dio *dio, struct dio_submit *sdio,
			   struct buffer_head *map_bh)
{
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	int ret;
	sector_t fs_startblk;	/* Into file, in filesystem-sized blocks */
	sector_t fs_endblk;	/* Into file, in filesystem-sized blocks */
	unsigned long fs_count;	/* Number of filesystem-sized blocks */
	int create;
	unsigned int i_blkbits = sdio->blkbits + sdio->blkfactor;
	loff_t i_size;

	/*
	 * If there was a memory error and we've overwritten all the
	 * mapped blocks then we can now return that memory error
	 */
	ret = dio->page_errors;
	if (ret == 0) {
		BUG_ON(sdio->block_in_file >= sdio->final_block_in_request);
		fs_startblk = sdio->block_in_file >> sdio->blkfactor;
		fs_endblk = (sdio->final_block_in_request - 1) >>
					sdio->blkfactor;
		fs_count = fs_endblk - fs_startblk + 1;

		map_bh->b_state = 0;
		map_bh->b_size = fs_count << i_blkbits;

		/*
		 * For writes that could fill holes inside i_size on a
		 * DIO_SKIP_HOLES filesystem we forbid block creations: only
		 * overwrites are permitted. We will return early to the caller
		 * once we see an unmapped buffer head returned, and the caller
		 * will fall back to buffered I/O.
		 *
		 * Otherwise the decision is left to the get_blocks method,
		 * which may decide to handle it or also return an unmapped
		 * buffer head.
		 */
		create = dio_op == REQ_OP_WRITE;
		if (dio->flags & DIO_SKIP_HOLES) {
			i_size = i_size_read(dio->inode);
			if (i_size && fs_startblk <= (i_size - 1) >> i_blkbits)
				create = 0;
		}

		ret = (*sdio->get_block)(dio->inode, fs_startblk,
						map_bh, create);

		/* Store for completion */
		dio->private = map_bh->b_private;

		if (ret == 0 && buffer_defer_completion(map_bh))
			ret = dio_set_defer_completion(dio);
	}
	return ret;
}

/*
 * There is no bio.  Make one now.
 */
static inline int dio_new_bio(struct dio *dio, struct dio_submit *sdio,
		sector_t start_sector, struct buffer_head *map_bh)
{
	sector_t sector;
	int ret, nr_pages;

	ret = dio_bio_reap(dio, sdio);
	if (ret)
		goto out;
	sector = start_sector << (sdio->blkbits - 9);
	nr_pages = bio_max_segs(sdio->pages_in_io);
	BUG_ON(nr_pages <= 0);
	dio_bio_alloc(dio, sdio, map_bh->b_bdev, sector, nr_pages);
	sdio->boundary = 0;
out:
	return ret;
}

/*
 * Attempt to put the current chunk of 'cur_page' into the current BIO.  If
 * that was successful then update final_block_in_bio and take a ref against
 * the just-added page.
 *
 * Return zero on success.  Non-zero means the caller needs to start a new BIO.
 */
static inline int dio_bio_add_page(struct dio *dio, struct dio_submit *sdio)
{
	int ret;

	ret = bio_add_page(sdio->bio, sdio->cur_page,
			sdio->cur_page_len, sdio->cur_page_offset);
	if (ret == sdio->cur_page_len) {
		/*
		 * Decrement count only, if we are done with this page
		 */
		if ((sdio->cur_page_len + sdio->cur_page_offset) == PAGE_SIZE)
			sdio->pages_in_io--;
		dio_pin_page(dio, sdio->cur_page);
		sdio->final_block_in_bio = sdio->cur_page_block +
			(sdio->cur_page_len >> sdio->blkbits);
		ret = 0;
	} else {
		ret = 1;
	}
	return ret;
}
		
/*
 * Put cur_page under IO.  The section of cur_page which is described by
 * cur_page_offset,cur_page_len is put into a BIO.  The section of cur_page
 * starts on-disk at cur_page_block.
 *
 * We take a ref against the page here (on behalf of its presence in the bio).
 *
 * The caller of this function is responsible for removing cur_page from the
 * dio, and for dropping the refcount which came from that presence.
 */
static inline int dio_send_cur_page(struct dio *dio, struct dio_submit *sdio,
		struct buffer_head *map_bh)
{
	int ret = 0;

	if (sdio->bio) {
		loff_t cur_offset = sdio->cur_page_fs_offset;
		loff_t bio_next_offset = sdio->logical_offset_in_bio +
			sdio->bio->bi_iter.bi_size;

		/*
		 * See whether this new request is contiguous with the old.
		 *
		 * Btrfs cannot handle having logically non-contiguous requests
		 * submitted.  For example if you have
		 *
		 * Logical:  [0-4095][HOLE][8192-12287]
		 * Physical: [0-4095]      [4096-8191]
		 *
		 * We cannot submit those pages together as one BIO.  So if our
		 * current logical offset in the file does not equal what would
		 * be the next logical offset in the bio, submit the bio we
		 * have.
		 */
		if (sdio->final_block_in_bio != sdio->cur_page_block ||
		    cur_offset != bio_next_offset)
			dio_bio_submit(dio, sdio);
	}

	if (sdio->bio == NULL) {
		ret = dio_new_bio(dio, sdio, sdio->cur_page_block, map_bh);
		if (ret)
			goto out;
	}

	if (dio_bio_add_page(dio, sdio) != 0) {
		dio_bio_submit(dio, sdio);
		ret = dio_new_bio(dio, sdio, sdio->cur_page_block, map_bh);
		if (ret == 0) {
			ret = dio_bio_add_page(dio, sdio);
			BUG_ON(ret != 0);
		}
	}
out:
	return ret;
}

/*
 * An autonomous function to put a chunk of a page under deferred IO.
 *
 * The caller doesn't actually know (or care) whether this piece of page is in
 * a BIO, or is under IO or whatever.  We just take care of all possible 
 * situations here.  The separation between the logic of do_direct_IO() and
 * that of submit_page_section() is important for clarity.  Please don't break.
 *
 * The chunk of page starts on-disk at blocknr.
 *
 * We perform deferred IO, by recording the last-submitted page inside our
 * private part of the dio structure.  If possible, we just expand the IO
 * across that page here.
 *
 * If that doesn't work out then we put the old page into the bio and add this
 * page to the dio instead.
 */
static inline int
submit_page_section(struct dio *dio, struct dio_submit *sdio, struct page *page,
		    unsigned offset, unsigned len, sector_t blocknr,
		    struct buffer_head *map_bh)
{
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	int ret = 0;
	int boundary = sdio->boundary;	/* dio_send_cur_page may clear it */

	if (dio_op == REQ_OP_WRITE) {
		/*
		 * Read accounting is performed in submit_bio()
		 */
		task_io_account_write(len);
	}

	/*
	 * Can we just grow the current page's presence in the dio?
	 */
	if (sdio->cur_page == page &&
	    sdio->cur_page_offset + sdio->cur_page_len == offset &&
	    sdio->cur_page_block +
	    (sdio->cur_page_len >> sdio->blkbits) == blocknr) {
		sdio->cur_page_len += len;
		goto out;
	}

	/*
	 * If there's a deferred page already there then send it.
	 */
	if (sdio->cur_page) {
		ret = dio_send_cur_page(dio, sdio, map_bh);
		dio_unpin_page(dio, sdio->cur_page);
		sdio->cur_page = NULL;
		if (ret)
			return ret;
	}

	dio_pin_page(dio, page);		/* It is in dio */
	sdio->cur_page = page;
	sdio->cur_page_offset = offset;
	sdio->cur_page_len = len;
	sdio->cur_page_block = blocknr;
	sdio->cur_page_fs_offset = sdio->block_in_file << sdio->blkbits;
out:
	/*
	 * If boundary then we want to schedule the IO now to
	 * avoid metadata seeks.
	 */
	if (boundary) {
		ret = dio_send_cur_page(dio, sdio, map_bh);
		if (sdio->bio)
			dio_bio_submit(dio, sdio);
		dio_unpin_page(dio, sdio->cur_page);
		sdio->cur_page = NULL;
	}
	return ret;
}

/*
 * If we are not writing the entire block and get_block() allocated
 * the block for us, we need to fill-in the unused portion of the
 * block with zeros. This happens only if user-buffer, fileoffset or
 * io length is not filesystem block-size multiple.
 *
 * `end' is zero if we're doing the start of the IO, 1 at the end of the
 * IO.
 */
static inline void dio_zero_block(struct dio *dio, struct dio_submit *sdio,
		int end, struct buffer_head *map_bh)
{
	unsigned dio_blocks_per_fs_block;
	unsigned this_chunk_blocks;	/* In dio_blocks */
	unsigned this_chunk_bytes;
	struct page *page;

	sdio->start_zero_done = 1;
	if (!sdio->blkfactor || !buffer_new(map_bh))
		return;

	dio_blocks_per_fs_block = 1 << sdio->blkfactor;
	this_chunk_blocks = sdio->block_in_file & (dio_blocks_per_fs_block - 1);

	if (!this_chunk_blocks)
		return;

	/*
	 * We need to zero out part of an fs block.  It is either at the
	 * beginning or the end of the fs block.
	 */
	if (end) 
		this_chunk_blocks = dio_blocks_per_fs_block - this_chunk_blocks;

	this_chunk_bytes = this_chunk_blocks << sdio->blkbits;

	page = ZERO_PAGE(0);
	if (submit_page_section(dio, sdio, page, 0, this_chunk_bytes,
				sdio->next_block_for_io, map_bh))
		return;

	sdio->next_block_for_io += this_chunk_blocks;
}

/*
 * Walk the user pages, and the file, mapping blocks to disk and generating
 * a sequence of (page,offset,len,block) mappings.  These mappings are injected
 * into submit_page_section(), which takes care of the next stage of submission
 *
 * Direct IO against a blockdev is different from a file.  Because we can
 * happily perform page-sized but 512-byte aligned IOs.  It is important that
 * blockdev IO be able to have fine alignment and large sizes.
 *
 * So what we do is to permit the ->get_block function to populate bh.b_size
 * with the size of IO which is permitted at this offset and this i_blkbits.
 *
 * For best results, the blockdev should be set up with 512-byte i_blkbits and
 * it should set b_size to PAGE_SIZE or more inside get_block().  This gives
 * fine alignment but still allows this function to work in PAGE_SIZE units.
 */
static int do_direct_IO(struct dio *dio, struct dio_submit *sdio,
			struct buffer_head *map_bh)
{
	const enum req_op dio_op = dio->opf & REQ_OP_MASK;
	const unsigned blkbits = sdio->blkbits;
	const unsigned i_blkbits = blkbits + sdio->blkfactor;
	int ret = 0;

	while (sdio->block_in_file < sdio->final_block_in_request) {
		struct page *page;
		size_t from, to;

		page = dio_get_page(dio, sdio);
		if (IS_ERR(page)) {
			ret = PTR_ERR(page);
			goto out;
		}
		from = sdio->head ? 0 : sdio->from;
		to = (sdio->head == sdio->tail - 1) ? sdio->to : PAGE_SIZE;
		sdio->head++;

		while (from < to) {
			unsigned this_chunk_bytes;	/* # of bytes mapped */
			unsigned this_chunk_blocks;	/* # of blocks */
			unsigned u;

			if (sdio->blocks_available == 0) {
				/*
				 * Need to go and map some more disk
				 */
				unsigned long blkmask;
				unsigned long dio_remainder;

				ret = get_more_blocks(dio, sdio, map_bh);
				if (ret) {
					dio_unpin_page(dio, page);
					goto out;
				}
				if (!buffer_mapped(map_bh))
					goto do_holes;

				sdio->blocks_available =
						map_bh->b_size >> blkbits;
				sdio->next_block_for_io =
					map_bh->b_blocknr << sdio->blkfactor;
				if (buffer_new(map_bh)) {
					clean_bdev_aliases(
						map_bh->b_bdev,
						map_bh->b_blocknr,
						map_bh->b_size >> i_blkbits);
				}

				if (!sdio->blkfactor)
					goto do_holes;

				blkmask = (1 << sdio->blkfactor) - 1;
				dio_remainder = (sdio->block_in_file & blkmask);

				/*
				 * If we are at the start of IO and that IO
				 * starts partway into a fs-block,
				 * dio_remainder will be non-zero.  If the IO
				 * is a read then we can simply advance the IO
				 * cursor to the first block which is to be
				 * read.  But if the IO is a write and the
				 * block was newly allocated we cannot do that;
				 * the start of the fs block must be zeroed out
				 * on-disk
				 */
				if (!buffer_new(map_bh))
					sdio->next_block_for_io += dio_remainder;
				sdio->blocks_available -= dio_remainder;
			}
do_holes:
			/* Handle holes */
			if (!buffer_mapped(map_bh)) {
				loff_t i_size_aligned;

				/* AKPM: eargh, -ENOTBLK is a hack */
				if (dio_op == REQ_OP_WRITE) {
					dio_unpin_page(dio, page);
					return -ENOTBLK;
				}

				/*
				 * Be sure to account for a partial block as the
				 * last block in the file
				 */
				i_size_aligned = ALIGN(i_size_read(dio->inode),
							1 << blkbits);
				if (sdio->block_in_file >=
						i_size_aligned >> blkbits) {
					/* We hit eof */
					dio_unpin_page(dio, page);
					goto out;
				}
				memzero_page(page, from, 1 << blkbits);
				sdio->block_in_file++;
				from += 1 << blkbits;
				dio->result += 1 << blkbits;
				goto next_block;
			}

			/*
			 * If we're performing IO which has an alignment which
			 * is finer than the underlying fs, go check to see if
			 * we must zero out the start of this block.
			 */
			if (unlikely(sdio->blkfactor && !sdio->start_zero_done))
				dio_zero_block(dio, sdio, 0, map_bh);

			/*
			 * Work out, in this_chunk_blocks, how much disk we
			 * can add to this page
			 */
			this_chunk_blocks = sdio->blocks_available;
			u = (to - from) >> blkbits;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			u = sdio->final_block_in_request - sdio->block_in_file;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			this_chunk_bytes = this_chunk_blocks << blkbits;
			BUG_ON(this_chunk_bytes == 0);

			if (this_chunk_blocks == sdio->blocks_available)
				sdio->boundary = buffer_boundary(map_bh);
			ret = submit_page_section(dio, sdio, page,
						  from,
						  this_chunk_bytes,
						  sdio->next_block_for_io,
						  map_bh);
			if (ret) {
				dio_unpin_page(dio, page);
				goto out;
			}
			sdio->next_block_for_io += this_chunk_blocks;

			sdio->block_in_file += this_chunk_blocks;
			from += this_chunk_bytes;
			dio->result += this_chunk_bytes;
			sdio->blocks_available -= this_chunk_blocks;
next_block:
			BUG_ON(sdio->block_in_file > sdio->final_block_in_request);
			if (sdio->block_in_file == sdio->final_block_in_request)
				break;
		}

		/* Drop the pin which was taken in get_user_pages() */
		dio_unpin_page(dio, page);
	}
out:
	return ret;
}

static inline int drop_refcount(struct dio *dio)
{
	int ret2;
	unsigned long flags;

	/*
	 * Sync will always be dropping the final ref and completing the
	 * operation.  AIO can if it was a broken operation described above or
	 * in fact if all the bios race to complete before we get here.  In
	 * that case dio_complete() translates the EIOCBQUEUED into the proper
	 * return code that the caller will hand to ->complete().
	 *
	 * This is managed by the bio_lock instead of being an atomic_t so that
	 * completion paths can drop their ref and use the remaining count to
	 * decide to wake the submission path atomically.
	 */
	spin_lock_irqsave(&dio->bio_lock, flags);
	ret2 = --dio->refcount;
	spin_unlock_irqrestore(&dio->bio_lock, flags);
	return ret2;
}

/*
 * This is a library function for use by filesystem drivers.
 *
 * The locking rules are governed by the flags parameter:
 *  - if the flags value contains DIO_LOCKING we use a fancy locking
 *    scheme for dumb filesystems.
 *    For writes this function is called under i_rwsem and returns with
 *    i_rwsem held, for reads, i_rwsem is not held on entry, but it is
 *    taken and dropped again before returning.
 *  - if the flags value does NOT contain DIO_LOCKING we don't use any
 *    internal locking but rather rely on the filesystem to synchronize
 *    direct I/O reads/writes versus each other and truncate.
 *
 * To help with locking against truncate we incremented the i_dio_count
 * counter before starting direct I/O, and decrement it once we are done.
 * Truncate can wait for it to reach zero to provide exclusion.  It is
 * expected that filesystem provide exclusion between new direct I/O
 * and truncates.  For DIO_LOCKING filesystems this is done by i_rwsem,
 * but other filesystems need to take care of this on their own.
 *
 * NOTE: if you pass "sdio" to anything by pointer make sure that function
 * is always inlined. Otherwise gcc is unable to split the structure into
 * individual fields and will generate much worse code. This is important
 * for the whole file.
 */
ssize_t __blockdev_direct_IO(struct kiocb *iocb, struct inode *inode,
		struct block_device *bdev, struct iov_iter *iter,
		get_block_t get_block, dio_iodone_t end_io,
		int flags)
{
	unsigned i_blkbits = READ_ONCE(inode->i_blkbits);
	unsigned blkbits = i_blkbits;
	unsigned blocksize_mask = (1 << blkbits) - 1;
	ssize_t retval = -EINVAL;
	const size_t count = iov_iter_count(iter);
	loff_t offset = iocb->ki_pos;
	const loff_t end = offset + count;
	struct dio *dio;
	struct dio_submit sdio = { NULL, };
	struct buffer_head map_bh = { 0, };
	struct blk_plug plug;
	unsigned long align = offset | iov_iter_alignment(iter);

	/* watch out for a 0 len io from a tricksy fs */
	if (iov_iter_rw(iter) == READ && !count)
		return 0;

	dio = kmem_cache_alloc(dio_cache, GFP_KERNEL);
	if (!dio)
		return -ENOMEM;
	/*
	 * Believe it or not, zeroing out the page array caused a .5%
	 * performance regression in a database benchmark.  So, we take
	 * care to only zero out what's needed.
	 */
	memset(dio, 0, offsetof(struct dio, pages));

	dio->flags = flags;
	if (dio->flags & DIO_LOCKING && iov_iter_rw(iter) == READ) {
		/* will be released by direct_io_worker */
		inode_lock(inode);
	}
	dio->is_pinned = iov_iter_extract_will_pin(iter);

	/* Once we sampled i_size check for reads beyond EOF */
	dio->i_size = i_size_read(inode);
	if (iov_iter_rw(iter) == READ && offset >= dio->i_size) {
		retval = 0;
		goto fail_dio;
	}

	if (align & blocksize_mask) {
		if (bdev)
			blkbits = blksize_bits(bdev_logical_block_size(bdev));
		blocksize_mask = (1 << blkbits) - 1;
		if (align & blocksize_mask)
			goto fail_dio;
	}

	if (dio->flags & DIO_LOCKING && iov_iter_rw(iter) == READ) {
		struct address_space *mapping = iocb->ki_filp->f_mapping;

		retval = filemap_write_and_wait_range(mapping, offset, end - 1);
		if (retval)
			goto fail_dio;
	}

	/*
	 * For file extending writes updating i_size before data writeouts
	 * complete can expose uninitialized blocks in dumb filesystems.
	 * In that case we need to wait for I/O completion even if asked
	 * for an asynchronous write.
	 */
	if (is_sync_kiocb(iocb))
		dio->is_async = false;
	else if (iov_iter_rw(iter) == WRITE && end > i_size_read(inode))
		dio->is_async = false;
	else
		dio->is_async = true;

	dio->inode = inode;
	if (iov_iter_rw(iter) == WRITE) {
		dio->opf = REQ_OP_WRITE | REQ_SYNC | REQ_IDLE;
		if (iocb->ki_flags & IOCB_NOWAIT)
			dio->opf |= REQ_NOWAIT;
	} else {
		dio->opf = REQ_OP_READ;
	}

	/*
	 * For AIO O_(D)SYNC writes we need to defer completions to a workqueue
	 * so that we can call ->fsync.
	 */
	if (dio->is_async && iov_iter_rw(iter) == WRITE) {
		retval = 0;
		if (iocb_is_dsync(iocb))
			retval = dio_set_defer_completion(dio);
		else if (!dio->inode->i_sb->s_dio_done_wq) {
			/*
			 * In case of AIO write racing with buffered read we
			 * need to defer completion. We can't decide this now,
			 * however the workqueue needs to be initialized here.
			 */
			retval = sb_init_dio_done_wq(dio->inode->i_sb);
		}
		if (retval)
			goto fail_dio;
	}

	/*
	 * Will be decremented at I/O completion time.
	 */
	inode_dio_begin(inode);

	sdio.blkbits = blkbits;
	sdio.blkfactor = i_blkbits - blkbits;
	sdio.block_in_file = offset >> blkbits;

	sdio.get_block = get_block;
	dio->end_io = end_io;
	sdio.final_block_in_bio = -1;
	sdio.next_block_for_io = -1;

	dio->iocb = iocb;

	spin_lock_init(&dio->bio_lock);
	dio->refcount = 1;

	dio->should_dirty = user_backed_iter(iter) && iov_iter_rw(iter) == READ;
	sdio.iter = iter;
	sdio.final_block_in_request = end >> blkbits;

	/*
	 * In case of non-aligned buffers, we may need 2 more
	 * pages since we need to zero out first and last block.
	 */
	if (unlikely(sdio.blkfactor))
		sdio.pages_in_io = 2;

	sdio.pages_in_io += iov_iter_npages(iter, INT_MAX);

	blk_start_plug(&plug);

	retval = do_direct_IO(dio, &sdio, &map_bh);
	if (retval)
		dio_cleanup(dio, &sdio);

	if (retval == -ENOTBLK) {
		/*
		 * The remaining part of the request will be
		 * handled by buffered I/O when we return
		 */
		retval = 0;
	}
	/*
	 * There may be some unwritten disk at the end of a part-written
	 * fs-block-sized block.  Go zero that now.
	 */
	dio_zero_block(dio, &sdio, 1, &map_bh);

	if (sdio.cur_page) {
		ssize_t ret2;

		ret2 = dio_send_cur_page(dio, &sdio, &map_bh);
		if (retval == 0)
			retval = ret2;
		dio_unpin_page(dio, sdio.cur_page);
		sdio.cur_page = NULL;
	}
	if (sdio.bio)
		dio_bio_submit(dio, &sdio);

	blk_finish_plug(&plug);

	/*
	 * It is possible that, we return short IO due to end of file.
	 * In that case, we need to release all the pages we got hold on.
	 */
	dio_cleanup(dio, &sdio);

	/*
	 * All block lookups have been performed. For READ requests
	 * we can let i_rwsem go now that its achieved its purpose
	 * of protecting us from looking up uninitialized blocks.
	 */
	if (iov_iter_rw(iter) == READ && (dio->flags & DIO_LOCKING))
		inode_unlock(dio->inode);

	/*
	 * The only time we want to leave bios in flight is when a successful
	 * partial aio read or full aio write have been setup.  In that case
	 * bio completion will call aio_complete.  The only time it's safe to
	 * call aio_complete is when we return -EIOCBQUEUED, so we key on that.
	 * This had *better* be the only place that raises -EIOCBQUEUED.
	 */
	BUG_ON(retval == -EIOCBQUEUED);
	if (dio->is_async && retval == 0 && dio->result &&
	    (iov_iter_rw(iter) == READ || dio->result == count))
		retval = -EIOCBQUEUED;
	else
		dio_await_completion(dio);

	if (drop_refcount(dio) == 0) {
		retval = dio_complete(dio, retval, DIO_COMPLETE_INVALIDATE);
	} else
		BUG_ON(retval != -EIOCBQUEUED);

	return retval;

fail_dio:
	if (dio->flags & DIO_LOCKING && iov_iter_rw(iter) == READ)
		inode_unlock(inode);

	kmem_cache_free(dio_cache, dio);
	return retval;
}
EXPORT_SYMBOL(__blockdev_direct_IO);

static __init int dio_init(void)
{
	dio_cache = KMEM_CACHE(dio, SLAB_PANIC);
	return 0;
}
module_init(dio_init)
