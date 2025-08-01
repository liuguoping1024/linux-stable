// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	Video for Linux Two
 *
 *	A generic video device interface for the LINUX operating system
 *	using a set of device structures/vectors for low level operations.
 *
 *	This file replaces the videodev.c file that comes with the
 *	regular kernel distribution.
 *
 * Author:	Bill Dirks <bill@thedirks.org>
 *		based on code by Alan Cox, <alan@cymru.net>
 */

/*
 * Video capture interface for Linux
 *
 *	A generic video device interface for the LINUX operating system
 *	using a set of device structures/vectors for low level operations.
 *
 * Author:	Alan Cox, <alan@lxorguk.ukuu.org.uk>
 *
 * Fixes:
 */

/*
 * Video4linux 1/2 integration by Justin Schoeman
 * <justin@suntiger.ee.up.ac.za>
 * 2.4 PROCFS support ported from 2.4 kernels by
 *  Iñaki García Etxebarria <garetxe@euskalnet.net>
 * Makefile fix by "W. Michael Petullo" <mike@flyn.org>
 * 2.4 devfs support ported from 2.4 kernels by
 *  Dan Merillat <dan@merillat.org>
 * Added Gerd Knorrs v4l1 enhancements (Justin Schoeman)
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include <linux/videodev2.h>

/*
 *
 *	V 4 L 2   D R I V E R   H E L P E R   A P I
 *
 */

/*
 *  Video Standard Operations (contributed by Michael Schimek)
 */

/* Helper functions for control handling			     */

/* Fill in a struct v4l2_queryctrl */
int v4l2_ctrl_query_fill(struct v4l2_queryctrl *qctrl, s32 _min, s32 _max, s32 _step, s32 _def)
{
	const char *name;
	s64 min = _min;
	s64 max = _max;
	u64 step = _step;
	s64 def = _def;

	v4l2_ctrl_fill(qctrl->id, &name, &qctrl->type,
		       &min, &max, &step, &def, &qctrl->flags);

	if (name == NULL)
		return -EINVAL;

	qctrl->minimum = min;
	qctrl->maximum = max;
	qctrl->step = step;
	qctrl->default_value = def;
	qctrl->reserved[0] = qctrl->reserved[1] = 0;
	strscpy(qctrl->name, name, sizeof(qctrl->name));
	return 0;
}
EXPORT_SYMBOL(v4l2_ctrl_query_fill);

/* Clamp x to be between min and max, aligned to a multiple of 2^align.  min
 * and max don't have to be aligned, but there must be at least one valid
 * value.  E.g., min=17,max=31,align=4 is not allowed as there are no multiples
 * of 16 between 17 and 31.  */
static unsigned int clamp_align(unsigned int x, unsigned int min,
				unsigned int max, unsigned int align)
{
	/* Bits that must be zero to be aligned */
	unsigned int mask = ~((1 << align) - 1);

	/* Clamp to aligned min and max */
	x = clamp(x, (min + ~mask) & mask, max & mask);

	/* Round to nearest aligned value */
	if (align)
		x = (x + (1 << (align - 1))) & mask;

	return x;
}

static unsigned int clamp_roundup(unsigned int x, unsigned int min,
				   unsigned int max, unsigned int alignment)
{
	x = clamp(x, min, max);
	if (alignment)
		x = round_up(x, alignment);

	return x;
}

void v4l_bound_align_image(u32 *w, unsigned int wmin, unsigned int wmax,
			   unsigned int walign,
			   u32 *h, unsigned int hmin, unsigned int hmax,
			   unsigned int halign, unsigned int salign)
{
	*w = clamp_align(*w, wmin, wmax, walign);
	*h = clamp_align(*h, hmin, hmax, halign);

	/* Usually we don't need to align the size and are done now. */
	if (!salign)
		return;

	/* How much alignment do we have? */
	walign = __ffs(*w);
	halign = __ffs(*h);
	/* Enough to satisfy the image alignment? */
	if (walign + halign < salign) {
		/* Max walign where there is still a valid width */
		unsigned int wmaxa = __fls(wmax ^ (wmin - 1));
		/* Max halign where there is still a valid height */
		unsigned int hmaxa = __fls(hmax ^ (hmin - 1));

		/* up the smaller alignment until we have enough */
		do {
			if (halign >= hmaxa ||
			    (walign <= halign && walign < wmaxa)) {
				*w = clamp_align(*w, wmin, wmax, walign + 1);
				walign = __ffs(*w);
			} else {
				*h = clamp_align(*h, hmin, hmax, halign + 1);
				halign = __ffs(*h);
			}
		} while (halign + walign < salign);
	}
}
EXPORT_SYMBOL_GPL(v4l_bound_align_image);

const void *
__v4l2_find_nearest_size_conditional(const void *array, size_t array_size,
				     size_t entry_size, size_t width_offset,
				     size_t height_offset, s32 width,
				     s32 height,
				     bool (*func)(const void *array,
						  size_t index,
						  const void *context),
				     const void *context)
{
	u32 error, min_error = U32_MAX;
	const void *best = NULL;
	size_t i;

	if (!array)
		return NULL;

	for (i = 0; i < array_size; i++, array += entry_size) {
		const u32 *entry_width = array + width_offset;
		const u32 *entry_height = array + height_offset;

		if (func && !func(array, i, context))
			continue;

		error = abs(*entry_width - width) + abs(*entry_height - height);
		if (error > min_error)
			continue;

		min_error = error;
		best = array;
		if (!error)
			break;
	}

	return best;
}
EXPORT_SYMBOL_GPL(__v4l2_find_nearest_size_conditional);

int v4l2_g_parm_cap(struct video_device *vdev,
		    struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_subdev_frame_interval ival = { 0 };
	int ret;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (vdev->device_caps & V4L2_CAP_READWRITE)
		a->parm.capture.readbuffers = 2;
	if (v4l2_subdev_has_op(sd, pad, get_frame_interval))
		a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	ret = v4l2_subdev_call_state_active(sd, pad, get_frame_interval, &ival);
	if (!ret)
		a->parm.capture.timeperframe = ival.interval;
	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_g_parm_cap);

int v4l2_s_parm_cap(struct video_device *vdev,
		    struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_subdev_frame_interval ival = {
		.interval = a->parm.capture.timeperframe
	};
	int ret;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	memset(&a->parm, 0, sizeof(a->parm));
	if (vdev->device_caps & V4L2_CAP_READWRITE)
		a->parm.capture.readbuffers = 2;
	else
		a->parm.capture.readbuffers = 0;

	if (v4l2_subdev_has_op(sd, pad, get_frame_interval))
		a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	ret = v4l2_subdev_call_state_active(sd, pad, set_frame_interval, &ival);
	if (!ret)
		a->parm.capture.timeperframe = ival.interval;
	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_s_parm_cap);

const struct v4l2_format_info *v4l2_format_info(u32 format)
{
	static const struct v4l2_format_info formats[] = {
		/* RGB formats */
		{ .format = V4L2_PIX_FMT_BGR24,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB24,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_HSV24,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGR32,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_XBGR32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGRX32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB32,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_XRGB32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGBX32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_HSV32,   .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_ARGB32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGBA32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_ABGR32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGRA32,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB565,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB565X, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB555,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGR666,  .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGR48_12, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 6, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_BGR48, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 6, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGB48, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 6, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_ABGR64_12, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 8, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGBA1010102, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RGBX1010102, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_ARGB2101010, .pixel_enc = V4L2_PIXEL_ENC_RGB, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },

		/* YUV packed formats */
		{ .format = V4L2_PIX_FMT_YUYV,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YVYU,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_UYVY,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_VYUY,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_Y210,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_Y212,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_Y216,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 4, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YUV48_12, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 6, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_MT2110T, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 5, 10, 0, 0 }, .bpp_div = { 4, 4, 1, 1 }, .hdiv = 2, .vdiv = 2,
		  .block_w = { 16, 8, 0, 0 }, .block_h = { 32, 16, 0, 0 }},
		{ .format = V4L2_PIX_FMT_MT2110R, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 5, 10, 0, 0 }, .bpp_div = { 4, 4, 1, 1 }, .hdiv = 2, .vdiv = 2,
		  .block_w = { 16, 8, 0, 0 }, .block_h = { 32, 16, 0, 0 }},

		/* YUV planar formats */
		{ .format = V4L2_PIX_FMT_NV12,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV21,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV15,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 5, 10, 0, 0 }, .bpp_div = { 4, 4, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV16,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_NV61,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_NV20,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 5, 10, 0, 0 }, .bpp_div = { 4, 4, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_NV24,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_NV42,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_P010,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 2, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_P012,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 2, 4, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },

		{ .format = V4L2_PIX_FMT_YUV410,  .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 4, .vdiv = 4 },
		{ .format = V4L2_PIX_FMT_YVU410,  .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 4, .vdiv = 4 },
		{ .format = V4L2_PIX_FMT_YUV411P, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 4, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YUV420,  .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_YVU420,  .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_YUV422P, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_GREY,    .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },

		/* Tiled YUV formats */
		{ .format = V4L2_PIX_FMT_NV12_4L4, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV15_4L4, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 5, 10, 0, 0 }, .bpp_div = { 4, 4, 1, 1 }, .hdiv = 2, .vdiv = 2,
		  .block_w = { 4, 2, 0, 0 }, .block_h = { 1, 1, 0, 0 }},
		{ .format = V4L2_PIX_FMT_P010_4L4, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 1, .comp_planes = 2, .bpp = { 2, 4, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },

		/* YUV planar formats, non contiguous variant */
		{ .format = V4L2_PIX_FMT_YUV420M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_YVU420M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_YUV422M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YVU422M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YUV444M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_YVU444M, .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 3, .comp_planes = 3, .bpp = { 1, 1, 1, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },

		{ .format = V4L2_PIX_FMT_NV12M,   .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV21M,   .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },
		{ .format = V4L2_PIX_FMT_NV16M,   .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_NV61M,   .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_P012M,   .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 2, 4, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2 },

		/* Tiled YUV formats, non contiguous variant */
		{ .format = V4L2_PIX_FMT_NV12MT,        .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2,
		  .block_w = { 64, 32, 0, 0 },	.block_h = { 32, 16, 0, 0 }},
		{ .format = V4L2_PIX_FMT_NV12MT_16X16,  .pixel_enc = V4L2_PIXEL_ENC_YUV, .mem_planes = 2, .comp_planes = 2, .bpp = { 1, 2, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 2, .vdiv = 2,
		  .block_w = { 16,  8, 0, 0 },	.block_h = { 16,  8, 0, 0 }},

		/* Bayer RGB formats */
		{ .format = V4L2_PIX_FMT_SBGGR8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR10,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG10,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG10,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB10,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR10P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 5, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG10P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 5, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG10P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 5, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB10P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 5, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR10ALAW8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG10ALAW8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG10ALAW8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB10ALAW8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR10DPCM8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG10DPCM8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG10DPCM8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB10DPCM8,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 1, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR12,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG12,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG12,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB12,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR12P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 2, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG12P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 2, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG12P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 2, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB12P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 3, 0, 0, 0 }, .bpp_div = { 2, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR14,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG14,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG14,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB14,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR14P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 7, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG14P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 7, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG14P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 7, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB14P,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 7, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SBGGR16,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGBRG16,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SGRBG16,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_SRGGB16,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 2, 0, 0, 0 }, .bpp_div = { 1, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },

		/* Renesas Camera Data Receiver Unit formats, bayer order agnostic */
		{ .format = V4L2_PIX_FMT_RAW_CRU10,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 8, 0, 0, 0 }, .bpp_div = { 6, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RAW_CRU12,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 8, 0, 0, 0 }, .bpp_div = { 5, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RAW_CRU14,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 8, 0, 0, 0 }, .bpp_div = { 4, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
		{ .format = V4L2_PIX_FMT_RAW_CRU20,	.pixel_enc = V4L2_PIXEL_ENC_BAYER, .mem_planes = 1, .comp_planes = 1, .bpp = { 8, 0, 0, 0 }, .bpp_div = { 3, 1, 1, 1 }, .hdiv = 1, .vdiv = 1 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i)
		if (formats[i].format == format)
			return &formats[i];
	return NULL;
}
EXPORT_SYMBOL(v4l2_format_info);

static inline unsigned int v4l2_format_block_width(const struct v4l2_format_info *info, int plane)
{
	if (!info->block_w[plane])
		return 1;
	return info->block_w[plane];
}

static inline unsigned int v4l2_format_block_height(const struct v4l2_format_info *info, int plane)
{
	if (!info->block_h[plane])
		return 1;
	return info->block_h[plane];
}

static inline unsigned int v4l2_format_plane_stride(const struct v4l2_format_info *info, int plane,
						    unsigned int width)
{
	unsigned int hdiv = plane ? info->hdiv : 1;
	unsigned int aligned_width =
		ALIGN(width, v4l2_format_block_width(info, plane));

	return DIV_ROUND_UP(aligned_width, hdiv) *
	       info->bpp[plane] / info->bpp_div[plane];
}

static inline unsigned int v4l2_format_plane_height(const struct v4l2_format_info *info, int plane,
						    unsigned int height)
{
	unsigned int vdiv = plane ? info->vdiv : 1;
	unsigned int aligned_height =
		ALIGN(height, v4l2_format_block_height(info, plane));

	return DIV_ROUND_UP(aligned_height, vdiv);
}

static inline unsigned int v4l2_format_plane_size(const struct v4l2_format_info *info, int plane,
						  unsigned int width, unsigned int height)
{
	return v4l2_format_plane_stride(info, plane, width) *
	       v4l2_format_plane_height(info, plane, height);
}

void v4l2_apply_frmsize_constraints(u32 *width, u32 *height,
				    const struct v4l2_frmsize_stepwise *frmsize)
{
	if (!frmsize)
		return;

	/*
	 * Clamp width/height to meet min/max constraints and round it up to
	 * macroblock alignment.
	 */
	*width = clamp_roundup(*width, frmsize->min_width, frmsize->max_width,
			       frmsize->step_width);
	*height = clamp_roundup(*height, frmsize->min_height, frmsize->max_height,
				frmsize->step_height);
}
EXPORT_SYMBOL_GPL(v4l2_apply_frmsize_constraints);

int v4l2_fill_pixfmt_mp(struct v4l2_pix_format_mplane *pixfmt,
			u32 pixelformat, u32 width, u32 height)
{
	const struct v4l2_format_info *info;
	struct v4l2_plane_pix_format *plane;
	int i;

	info = v4l2_format_info(pixelformat);
	if (!info)
		return -EINVAL;

	pixfmt->width = width;
	pixfmt->height = height;
	pixfmt->pixelformat = pixelformat;
	pixfmt->num_planes = info->mem_planes;

	if (info->mem_planes == 1) {
		plane = &pixfmt->plane_fmt[0];
		plane->bytesperline = v4l2_format_plane_stride(info, 0, width);
		plane->sizeimage = 0;

		for (i = 0; i < info->comp_planes; i++)
			plane->sizeimage +=
				v4l2_format_plane_size(info, i, width, height);
	} else {
		for (i = 0; i < info->comp_planes; i++) {
			plane = &pixfmt->plane_fmt[i];
			plane->bytesperline =
				v4l2_format_plane_stride(info, i, width);
			plane->sizeimage = plane->bytesperline *
				v4l2_format_plane_height(info, i, height);
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_fill_pixfmt_mp);

int v4l2_fill_pixfmt(struct v4l2_pix_format *pixfmt, u32 pixelformat,
		     u32 width, u32 height)
{
	const struct v4l2_format_info *info;
	int i;

	info = v4l2_format_info(pixelformat);
	if (!info)
		return -EINVAL;

	/* Single planar API cannot be used for multi plane formats. */
	if (info->mem_planes > 1)
		return -EINVAL;

	pixfmt->width = width;
	pixfmt->height = height;
	pixfmt->pixelformat = pixelformat;
	pixfmt->bytesperline = v4l2_format_plane_stride(info, 0, width);
	pixfmt->sizeimage = 0;

	for (i = 0; i < info->comp_planes; i++)
		pixfmt->sizeimage +=
			v4l2_format_plane_size(info, i, width, height);
	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_fill_pixfmt);

s64 __v4l2_get_link_freq_ctrl(struct v4l2_ctrl_handler *handler,
			      unsigned int mul, unsigned int div)
{
	struct v4l2_ctrl *ctrl;
	s64 freq;

	ctrl = v4l2_ctrl_find(handler, V4L2_CID_LINK_FREQ);
	if (ctrl) {
		struct v4l2_querymenu qm = { .id = V4L2_CID_LINK_FREQ };
		int ret;

		qm.index = v4l2_ctrl_g_ctrl(ctrl);

		ret = v4l2_querymenu(handler, &qm);
		if (ret)
			return -ENOENT;

		freq = qm.value;
	} else {
		if (!mul || !div)
			return -ENOENT;

		ctrl = v4l2_ctrl_find(handler, V4L2_CID_PIXEL_RATE);
		if (!ctrl)
			return -ENOENT;

		freq = div_u64(v4l2_ctrl_g_ctrl_int64(ctrl) * mul, div);

		pr_warn_once("%s: Link frequency estimated using pixel rate: result might be inaccurate\n",
			     __func__);
		pr_warn_once("%s: Consider implementing support for V4L2_CID_LINK_FREQ in the transmitter driver\n",
			     __func__);
	}

	return freq > 0 ? freq : -EINVAL;
}
EXPORT_SYMBOL_GPL(__v4l2_get_link_freq_ctrl);

#ifdef CONFIG_MEDIA_CONTROLLER
s64 __v4l2_get_link_freq_pad(struct media_pad *pad, unsigned int mul,
			     unsigned int div)
{
	struct v4l2_mbus_config mbus_config = {};
	struct v4l2_subdev *sd;
	int ret;

	sd = media_entity_to_v4l2_subdev(pad->entity);
	ret = v4l2_subdev_call(sd, pad, get_mbus_config, pad->index,
			       &mbus_config);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	if (mbus_config.link_freq)
		return mbus_config.link_freq;

	/*
	 * Fall back to using the link frequency control if the media bus config
	 * doesn't provide a link frequency.
	 */
	return __v4l2_get_link_freq_ctrl(sd->ctrl_handler, mul, div);
}
EXPORT_SYMBOL_GPL(__v4l2_get_link_freq_pad);
#endif /* CONFIG_MEDIA_CONTROLLER */

/*
 * Simplify a fraction using a simple continued fraction decomposition. The
 * idea here is to convert fractions such as 333333/10000000 to 1/30 using
 * 32 bit arithmetic only. The algorithm is not perfect and relies upon two
 * arbitrary parameters to remove non-significative terms from the simple
 * continued fraction decomposition. Using 8 and 333 for n_terms and threshold
 * respectively seems to give nice results.
 */
void v4l2_simplify_fraction(u32 *numerator, u32 *denominator,
		unsigned int n_terms, unsigned int threshold)
{
	u32 *an;
	u32 x, y, r;
	unsigned int i, n;

	an = kmalloc_array(n_terms, sizeof(*an), GFP_KERNEL);
	if (an == NULL)
		return;

	/*
	 * Convert the fraction to a simple continued fraction. See
	 * https://en.wikipedia.org/wiki/Continued_fraction
	 * Stop if the current term is bigger than or equal to the given
	 * threshold.
	 */
	x = *numerator;
	y = *denominator;

	for (n = 0; n < n_terms && y != 0; ++n) {
		an[n] = x / y;
		if (an[n] >= threshold) {
			if (n < 2)
				n++;
			break;
		}

		r = x - an[n] * y;
		x = y;
		y = r;
	}

	/* Expand the simple continued fraction back to an integer fraction. */
	x = 0;
	y = 1;

	for (i = n; i > 0; --i) {
		r = y;
		y = an[i-1] * y + x;
		x = r;
	}

	*numerator = y;
	*denominator = x;
	kfree(an);
}
EXPORT_SYMBOL_GPL(v4l2_simplify_fraction);

/*
 * Convert a fraction to a frame interval in 100ns multiples. The idea here is
 * to compute numerator / denominator * 10000000 using 32 bit fixed point
 * arithmetic only.
 */
u32 v4l2_fraction_to_interval(u32 numerator, u32 denominator)
{
	u32 multiplier;

	/* Saturate the result if the operation would overflow. */
	if (denominator == 0 ||
	    numerator/denominator >= ((u32)-1)/10000000)
		return (u32)-1;

	/*
	 * Divide both the denominator and the multiplier by two until
	 * numerator * multiplier doesn't overflow. If anyone knows a better
	 * algorithm please let me know.
	 */
	multiplier = 10000000;
	while (numerator > ((u32)-1)/multiplier) {
		multiplier /= 2;
		denominator /= 2;
	}

	return denominator ? numerator * multiplier / denominator : 0;
}
EXPORT_SYMBOL_GPL(v4l2_fraction_to_interval);

int v4l2_link_freq_to_bitmap(struct device *dev, const u64 *fw_link_freqs,
			     unsigned int num_of_fw_link_freqs,
			     const s64 *driver_link_freqs,
			     unsigned int num_of_driver_link_freqs,
			     unsigned long *bitmap)
{
	unsigned int i;

	*bitmap = 0;

	if (!num_of_fw_link_freqs) {
		dev_err(dev, "no link frequencies in firmware\n");
		return -ENODATA;
	}

	for (i = 0; i < num_of_fw_link_freqs; i++) {
		unsigned int j;

		for (j = 0; j < num_of_driver_link_freqs; j++) {
			if (fw_link_freqs[i] != driver_link_freqs[j])
				continue;

			dev_dbg(dev, "enabling link frequency %lld Hz\n",
				driver_link_freqs[j]);
			*bitmap |= BIT(j);
			break;
		}
	}

	if (!*bitmap) {
		dev_err(dev, "no matching link frequencies found\n");

		dev_dbg(dev, "specified in firmware:\n");
		for (i = 0; i < num_of_fw_link_freqs; i++)
			dev_dbg(dev, "\t%llu Hz\n", fw_link_freqs[i]);

		dev_dbg(dev, "driver supported:\n");
		for (i = 0; i < num_of_driver_link_freqs; i++)
			dev_dbg(dev, "\t%lld Hz\n", driver_link_freqs[i]);

		return -ENOENT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_link_freq_to_bitmap);
