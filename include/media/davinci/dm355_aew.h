/*
 * Copyright (C) 2009 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef DM355_AEW_DRIVER_H
#define DM355_AEW_DRIVER_H

#include <linux/ioctl.h>

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/fcntl.h>
#endif

/* Driver Range Constants*/
#define AEW_WINDOW_VERTICAL_COUNT_MIN		1
#define AEW_WINDOW_VERTICAL_COUNT_MAX		128
#define AEW_WINDOW_HORIZONTAL_COUNT_MIN		2
#define AEW_WINDOW_HORIZONTAL_COUNT_MAX		36
#define AEW_WINDOW_SIZE				18
#define AEW_WIDTH_MIN				8
#define AEW_WIDTH_MAX				256
#define AEW_AVELMT_MAX				1023
#define AEW_HZ_LINEINCR_MIN			2
#define AEW_HZ_LINEINCR_MAX			32
#define AEW_VT_LINEINCR_MIN			2
#define AEW_VT_LINEINCR_MAX			32
#define AEW_HEIGHT_MIN				2
#define AEW_HEIGHT_MAX				256
#define AEW_HZSTART_MIN				0
#define AEW_HZSTART_MAX				4095
#define AEW_VTSTART_MIN				0
#define AEW_VTSTART_MAX				4095
#define AEW_BLKWINHEIGHT_MIN			2
#define AEW_BLKWINHEIGHT_MAX			256
#define AEW_BLKWINVTSTART_MIN			0
#define AEW_BLKWINVTSTART_MAX			4095

#ifdef __KERNEL__

/* Device Constants*/
#define AEW_NR_DEVS		1
#define DEVICE_NAME		"dm355_aew"
#define AEW_MAJOR_NUMBER	0
#define AEW_IOC_MAXNR		4
#define AEW_TIMEOUT		(300 * HZ / 1000)
#endif

/* List of ioctls */
#define AEW_MAGIC_NO    'e'
#define AEW_S_PARAM	_IOWR(AEW_MAGIC_NO , 1 , struct aew_configuration)
#define AEW_G_PARAM	_IOWR(AEW_MAGIC_NO , 2 , struct aew_configuration)
#define AEW_GET_STAT	_IOWR(AEW_MAGIC_NO, 5, struct aew_statdata)

/*Enum for device usage*/
enum aew_In_use {
	AEW_NOT_IN_USE = 0,	/* Device is not in use */
	AEW_IN_USE = 1		/* Device in use */
};

/*Enum for Enable/Disable specific feature*/
enum aew_alaw_enable {
	H3A_AEW_ENABLE = 1,
	H3A_AEW_DISABLE = 0
};

enum aew_config_flag {
	H3A_AEW_CONFIG_NOT_DONE,
	H3A_AEW_CONFIG
};

/* Contains the information regarding Window Structure in AEW Engine */
struct aew_window {
	unsigned int width;		/* Width of the window */
	unsigned int height;		/* Height of the window */
	unsigned int hz_start;		/* Horizontal Start of the window */
	unsigned int vt_start;		/* Vertical Start of the window */
	unsigned int hz_cnt;		/* Horizontal Count */
	unsigned int vt_cnt;		/* Vertical Count */
	unsigned int hz_line_incr;	/* Horizontal Line Increment */
	unsigned int vt_line_incr;	/* Vertical Line Increment */
};

/* Contains the information regarding the AEW Black Window Structure*/
struct aew_black_window {
	unsigned int height;	/* Height of the Black Window */
	unsigned int vt_start;	/* Vertical Start of the black Window */
};

enum aew_input_src_t {
	AEW_CCDC = 0,
	AEW_SDRAM = 1
};

/* Contains configuration required for setup of AEW engine*/
struct aew_configuration {
	enum aew_alaw_enable alaw_enable;		/* A-law status */
	int saturation_limit;			/* Saturation Limit */
	struct aew_window window_config;	/* Window for AEW Engine */
	struct aew_black_window blackwindow_config;	/* Black Window */
};

struct aew_statdata {
	void *buffer;
	int buf_length;
};

#ifdef __KERNEL__
/* Contains information about device structure of AEW*/
struct aew_device {
	enum aew_In_use in_use;			/* Driver usage flag */
	struct aew_configuration *config;	/* Device configuration */
	void *buff_old;			/* Contains latest statistics */
	void *buff_curr;		/* Buffer in which HW will */

	/*
	 * Fill the statistics
	 * or HW is already filling
	 * statistics
	 */

	void *buff_app;		/* Buffer which will be passed */

	int buffer_filled;	/* Flag indicates statistics */

	unsigned int size_window;
	wait_queue_head_t aew_wait_queue;
	struct mutex read_blocked;

	/* Flag indicates Engine is configured */
	enum aew_config_flag aew_config;
};

int aew_hardware_setup(void);
int aew_validate_parameters(void);
#endif
#endif
