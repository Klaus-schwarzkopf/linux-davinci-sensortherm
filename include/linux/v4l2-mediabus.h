/*
 * Media Bus API header
 *
 * Copyright (C) 2009, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_V4L2_MEDIABUS_H
#define __LINUX_V4L2_MEDIABUS_H

#include <linux/types.h>
#include <linux/videodev2.h>

/*
 * These pixel codes uniquely identify data formats on the media bus. Mostly
 * they correspond to similarly named V4L2_PIX_FMT_* formats, format 0 is
 * reserved, V4L2_MBUS_FMT_FIXED shall be used by host-client pairs, where the
 * data format is fixed. Additionally, "2X8" means that one pixel is transferred
 * in two 8-bit samples, "BE" or "LE" specify in which order those samples are
 * transferred over the bus: "LE" means that the least significant bits are
 * transferred first, "BE" means that the most significant bits are transferred
 * first, and "PADHI" and "PADLO" define which bits - low or high, in the
 * incomplete high byte, are filled with padding bits.
 */
enum v4l2_mbus_pixelcode {
	V4L2_MBUS_FMT_FIXED = 1,
	V4L2_MBUS_FMT_YUYV8_2X8,
	V4L2_MBUS_FMT_YVYU8_2X8,
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_VYUY8_2X8,
	V4L2_MBUS_FMT_YVYU10_2X10,
	V4L2_MBUS_FMT_YUYV10_2X10,
	V4L2_MBUS_FMT_YVYU10_1X20,
	V4L2_MBUS_FMT_YUYV10_1X20,
	V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,
	V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE,
	V4L2_MBUS_FMT_RGB565_2X8_LE,
	V4L2_MBUS_FMT_RGB565_2X8_BE,
	V4L2_MBUS_FMT_BGR565_2X8_LE,
	V4L2_MBUS_FMT_BGR565_2X8_BE,
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_GREY8_1X8,
	V4L2_MBUS_FMT_Y10_1X10,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE,
	V4L2_MBUS_FMT_SGRBG8_1X8,
	V4L2_MBUS_FMT_SBGGR12_1X12,
	V4L2_MBUS_FMT_YUYV8_1_5X8,
	V4L2_MBUS_FMT_YVYU8_1_5X8,
	V4L2_MBUS_FMT_UYVY8_1_5X8,
	V4L2_MBUS_FMT_VYUY8_1_5X8,
};

/**
 * struct v4l2_mbus_framefmt - frame format on the media bus
 * @width:	frame width
 * @height:	frame height
 * @code:	data format code (from enum v4l2_mbus_pixelcode)
 * @field:	used interlacing type (from enum v4l2_field)
 * @colorspace:	colorspace of the data (from enum v4l2_colorspace)
 */
struct v4l2_mbus_framefmt {
	__u32			width;
	__u32			height;
	__u32			code;
	__u32			field;
	__u32			colorspace;
	__u32			reserved[7];
};

#endif
