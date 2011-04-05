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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <media/v4l2-device.h>
#include <linux/device.h>
#include <media/davinci/dm355_aew.h>
#include <media/davinci/dm355_aew_hw.h>
#include <media/davinci/vpss.h>
#include <media/davinci/vpfe_aew.h>

#define DRIVERNAME  "DM355AEW"

struct aew_device *aew_dev_configptr;
struct device *aewdev;

int aew_validate_parameters(void)
{
	int result = 0;

	/* Check horizontal Count */
	if ((aew_dev_configptr->config->window_config.hz_cnt <
		AEW_WINDOW_HORIZONTAL_COUNT_MIN) ||
		(aew_dev_configptr->config->window_config.hz_cnt >
		AEW_WINDOW_HORIZONTAL_COUNT_MAX)) {
			dev_err(aewdev, "\n Horizontal Count"
				" is incorrect");
			result = -EINVAL;
	}

	/* Check Vertical Count */
	if ((aew_dev_configptr->config->window_config.vt_cnt <
		AEW_WINDOW_VERTICAL_COUNT_MIN) ||
		(aew_dev_configptr->config->window_config.vt_cnt >
		AEW_WINDOW_VERTICAL_COUNT_MAX)) {
			dev_err(aewdev, "\n Vertical Count"
				" is incorrect");
			result = -EINVAL;
	}

	/* Check line increment */
	if ((AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->window_config.
		hz_line_incr)) ||
		(aew_dev_configptr->config->window_config.hz_line_incr <
		AEW_HZ_LINEINCR_MIN) ||
		(aew_dev_configptr->config->window_config.hz_line_incr >
		AEW_HZ_LINEINCR_MAX)) {
			dev_err(aewdev, "\n Invalid Parameters");
			dev_err(aewdev, "\nHorizontal Line"
				" Increment is incorrect");
			result = -EINVAL;
	}

	/* Check line increment */
	if ((AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->window_config.
		vt_line_incr)) ||
		(aew_dev_configptr->config->window_config.vt_line_incr <
		AEW_VT_LINEINCR_MIN) ||
		(aew_dev_configptr->config->window_config.vt_line_incr >
		AEW_VT_LINEINCR_MAX)) {
			dev_err(aewdev, "\nInvalid Parameters");
			dev_err(aewdev, "\nVertical Line"
				" Increment is incorrect");
			result = -EINVAL;
	}

	/* Check width */
	if ((AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->window_config.
		width)) ||
		(aew_dev_configptr->config->window_config.width <
		AEW_WIDTH_MIN) ||
		(aew_dev_configptr->config->window_config.width >
		AEW_WIDTH_MAX)) {
			dev_err(aewdev, "\nWidth is incorrect");

			result = -EINVAL;
	}

	/* Check Height */
	if ((AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->window_config.
		height)) || (aew_dev_configptr->config->window_config.height <
		AEW_HEIGHT_MIN) ||
		(aew_dev_configptr->config->window_config.height >
		AEW_HEIGHT_MAX)) {
		dev_err(aewdev, "\nheight incorrect");
		result = -EINVAL;
	}

	/* Check Horizontal Start */
	if ((aew_dev_configptr->config->window_config.hz_start <
		AEW_HZSTART_MIN) ||
		(aew_dev_configptr->config->window_config.hz_start >
		AEW_HZSTART_MAX)) {
			dev_err(aewdev, "\n horizontal start"
				" is  incorrect");
			result = -EINVAL;
	}

	if ((aew_dev_configptr->config->window_config.vt_start >
	     AEW_VTSTART_MAX)) {
		dev_err(aewdev, "\n Vertical start is  incorrect");
		result = -EINVAL;
	}

	if ((aew_dev_configptr->config->alaw_enable > H3A_AEW_ENABLE) ||
		(aew_dev_configptr->config->alaw_enable < H3A_AEW_DISABLE)) {
		dev_err(aewdev, "\n A Law setting is incorrect");
		result = -EINVAL;
	}

	if (aew_dev_configptr->config->saturation_limit > AEW_AVELMT_MAX) {
		dev_err(aewdev, "\n Saturation Limit is incorrect");
		result = -EINVAL;
	}

	/* Check Black Window Height */
	if (AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->blackwindow_config.
		height) ||
		(aew_dev_configptr->config->blackwindow_config.height <
		AEW_BLKWINHEIGHT_MIN) ||
		(aew_dev_configptr->config->blackwindow_config.height >
		AEW_BLKWINHEIGHT_MAX)) {
			dev_err(aewdev, "\n Black Window height incorrect");
			result = -EINVAL;
	}

	/* Check Black Window Height */
	if ((AEW_NOT_EVEN ==
		AEW_CHECK_EVEN(aew_dev_configptr->config->blackwindow_config.
		height)) ||
		(aew_dev_configptr->config->blackwindow_config.vt_start <
		AEW_BLKWINVTSTART_MIN) ||
		(aew_dev_configptr->config->blackwindow_config.vt_start >
		AEW_BLKWINVTSTART_MAX)) {
			dev_err(aewdev, "\n Black Window vertical"
				" Start is incorrect");
			result = -EINVAL;
	}

	if (((aew_dev_configptr->config->window_config.vt_cnt) *
		(aew_dev_configptr->config->window_config.height) +
		(aew_dev_configptr->config->window_config.vt_start)) > 156) {
			dev_err(aewdev, "\n Only 156 Lines are"
				" supported for CCDC mode");
			dev_err(aewdev, "\n Vertical count * Height +"
				" vertical Start should not exceed 156");
		result = -EINVAL;
	}

	return result;
}

/* inline function to free reserver pages */
inline void aew_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long tempaddr;
	unsigned long size;

	tempaddr = addr;
	if (!addr)
		return;

	size = PAGE_SIZE << (get_order(bufsize));
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	free_pages(tempaddr, get_order(bufsize));
}

/* Function to perform hardware Configuration */
int aew_hardware_setup(void)
{
	int result;
	int buff_size = 0;
	unsigned long adr;
	unsigned long size;
	unsigned int busyaew;

	/* Get the value of PCR register */
	busyaew = AEW_GET_PCR;

	/* Mask with BUSYAF bit */
	busyaew = busyaew & AEW_BUSYAEWB;

	/* Shift it 18 times to get value of 1 or 0 */
	busyaew = busyaew >> AEW_BUSYAEWB_SHIFT;

	/* If H3A Engine is busy then return */
	if (busyaew == 1) {
		dev_err(aewdev, "\n Error : AEW Engine is busy");
		return -EBUSY;
	}

	result = aew_validate_parameters();

	dev_dbg(aewdev, "Result =  %d\n", result);
	if (result < 0) {
		dev_err(aewdev, "Error : Parameters are incorrect\n");
		return result;
	}

	/* Deallocate the previously allocated buffers */
	if (aew_dev_configptr->buff_old)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_old,
			aew_dev_configptr->size_window);

	if (aew_dev_configptr->buff_curr)
		aew_free_pages((unsigned long)aew_dev_configptr->
			buff_curr, aew_dev_configptr->size_window);

	if (aew_dev_configptr->buff_app)
		aew_free_pages((unsigned long)aew_dev_configptr->
			buff_app, aew_dev_configptr->size_window);

	/*
	 * Allocat the buffers as per the new buffer size
	 * Allocate memory for old buffer
	 */
	buff_size = (aew_dev_configptr->config->window_config.hz_cnt) *
			(aew_dev_configptr->config->window_config.vt_cnt) *
			AEW_WINDOW_SIZE;

	aew_dev_configptr->buff_old = (void *)__get_free_pages(GFP_KERNEL |
					GFP_DMA, get_order(buff_size));

	if (aew_dev_configptr->buff_old == NULL)
		return -ENOMEM;

	/* Make pages reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_old;
	size = PAGE_SIZE << (get_order(buff_size));

	while (size > 0) {
		/*
		 * Make sure the frame buffers
		 * are never swapped out of memory
		 */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/*Allocate memory for current buffer */
	aew_dev_configptr->buff_curr = (void *)__get_free_pages(GFP_KERNEL |
					GFP_DMA, get_order(buff_size));

	if (aew_dev_configptr->buff_curr == NULL) {
		/* Free all  buffer that are allocated */
		if (aew_dev_configptr->buff_old)
			aew_free_pages((unsigned long)aew_dev_configptr->
					buff_old, buff_size);
		return -ENOMEM;
	}

	/* Make pages reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_curr;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/*
		 * Make sure the frame buffers
		 * are never swapped out of memory
		 */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/* Allocate memory for application buffer */
	aew_dev_configptr->buff_app = (void *)__get_free_pages(GFP_KERNEL |
					GFP_DMA, get_order(buff_size));

	if (aew_dev_configptr->buff_app == NULL) {
		/* Free all buffer that were allocated previously */
		if (aew_dev_configptr->buff_old)
			aew_free_pages((unsigned long)aew_dev_configptr->
				       buff_old, buff_size);
		if (aew_dev_configptr->buff_curr)
			aew_free_pages((unsigned long)aew_dev_configptr->
				       buff_curr, buff_size);
		return -ENOMEM;
	}

	/* Make pages reserved so that they will be swapped out */
	adr = (unsigned long)aew_dev_configptr->buff_app;
	size = PAGE_SIZE << (get_order(buff_size));
	while (size > 0) {
		/*
		 * Make sure the frame buffers
		 * are never swapped out of memory
		 */
		SetPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/* Set the registers */
	aew_register_setup(aew_dev_configptr);
	aew_dev_configptr->size_window = buff_size;
	aew_dev_configptr->aew_config = H3A_AEW_CONFIG;

	return 0;
}

int dm355_aew_open(void)
{
	/* Return if Device is in use (Single Channel Support is provided) */
	if (aew_dev_configptr->in_use == AEW_IN_USE)
		return -EBUSY;

	/* Set the aew_dev_configptr structure */
	aew_dev_configptr->config = NULL;

	/* Allocate memory for configuration  structure of this channel */
	aew_dev_configptr->config = (struct aew_configuration *)
		kmalloc(sizeof(struct aew_configuration), GFP_KERNEL);

	if (aew_dev_configptr->config == NULL) {
		dev_err(aewdev, "Error : Kmalloc fail\n");
		return -ENOMEM;
	}

	/* Initiaze the wait queue */
	init_waitqueue_head(&(aew_dev_configptr->aew_wait_queue));

	/* Device is in use */
	aew_dev_configptr->in_use = AEW_IN_USE;

	/* No Hardware Set up done */
	aew_dev_configptr->aew_config = H3A_AEW_CONFIG_NOT_DONE;

	/* No statistics are available */
	aew_dev_configptr->buffer_filled = 0;

	/* Set Window Size to 0 */
	aew_dev_configptr->size_window = 0;

	/* Initialize the mutex */
	mutex_init(&(aew_dev_configptr->read_blocked));

	return 0;
}
EXPORT_SYMBOL(dm355_aew_open);


/* This Function is called when driver is closed */
int dm355_aew_release(void)
{
	aew_engine_setup(0);

	/* The Application has closed device so device is not in use */
	aew_dev_configptr->in_use = AEW_NOT_IN_USE;

	/* Release memory for configuration structure of this channel */
	kfree(aew_dev_configptr->config);

	/* Free Old Buffer */
	if (aew_dev_configptr->buff_old)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_old,
			aew_dev_configptr->size_window);

	/* Free Current Buffer */
	if (aew_dev_configptr->buff_curr)
		aew_free_pages((unsigned long)aew_dev_configptr->
			buff_curr, aew_dev_configptr->size_window);

	/* Free Application Buffer */
	if (aew_dev_configptr->buff_app)
		aew_free_pages((unsigned long)aew_dev_configptr->buff_app,
			aew_dev_configptr->size_window);

	aew_dev_configptr->buff_old	= NULL;
	aew_dev_configptr->buff_curr	= NULL;
	aew_dev_configptr->config	= NULL;
	aew_dev_configptr->buff_app	= NULL;

	return 0;
}
EXPORT_SYMBOL(dm355_aew_release);


/*
 * This function will process IOCTL commands sent by the application and
 * control the devices IO operations.
 */
int dm355_aew_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	/* Stores Previous Configurations */
	struct aew_configuration aewconfig = *(aew_dev_configptr->config);
	struct aew_statdata *stat_data = (struct aew_statdata *)arg;
	void *buffer_temp;
	int result = 0;

	/* Switch according to IOCTL command */
	switch (cmd) {
		/*
		 * This ioctl is used to perform hardware set up
		 * and will set all the regiseters for AF engine
		 */
	case AEW_S_PARAM:

		/* Copy config structure passed by user */
		memcpy(aew_dev_configptr->config,
				   (struct aew_configuration *)arg,
		       sizeof(struct aew_configuration));

		/* Call aew_hardware_setup to perform register configuration */
		result = aew_hardware_setup();
		if (!result) {
			/*
			 * Hardware Set up is successful
			 * Return the no of bytes required for buffer
			 */
			result = aew_dev_configptr->size_window;
		} else {
			/* Change Configuration Structure to original */
			*(aew_dev_configptr->config) = aewconfig;
			dev_err(aewdev, "Error : AEW_S_PARAM  failed\n");
		}

		break;

		/* This ioctl is used to return parameters in user space */
	case AEW_G_PARAM:
		if (aew_dev_configptr->aew_config == H3A_AEW_CONFIG) {
			memcpy((struct aew_configuration *)arg,
			     aew_dev_configptr->config,
			       sizeof(struct aew_configuration));
			result = aew_dev_configptr->size_window;
		} else {
			dev_err(aewdev,
				"Error : AEW Hardware is not configured.\n");
			result = -EINVAL;
		}
		break;

	case AEW_GET_STAT:
		/* Implement the read  functionality */
		if (aew_dev_configptr->buffer_filled != 1) {
			printk(KERN_ERR "buffer empty!\n");
			return -EINVAL;
		}
		if (stat_data->buf_length < aew_dev_configptr->size_window) {
			printk(KERN_ERR "invalid length\n");
			return -EINVAL;
		}

		/* Disable the interrupts and then swap the buffers */
		dev_dbg(aewdev, "READING............\n");
		disable_irq(4);

		/* New Statistics are availaible */
		aew_dev_configptr->buffer_filled = 0;

		/* Swap application buffer and old buffer */
		buffer_temp = aew_dev_configptr->buff_old;
		aew_dev_configptr->buff_old = aew_dev_configptr->buff_app;
		aew_dev_configptr->buff_app = buffer_temp;

		/* Interrupts are enabled */
		enable_irq(4);

		/*
		* Copy the entire statistics located in application
		* buffer to user space
		*/
		memcpy(stat_data->buffer, aew_dev_configptr->buff_app,
				aew_dev_configptr->size_window);

		result = aew_dev_configptr->size_window;
		dev_dbg(aewdev, "Reading Done........................\n");

		break;
		/* Invalid Command */
	default:
		dev_err(aewdev, "Error: It should not come here!!\n");
		result = -ENOTTY;
		break;
	}

	/*Release the mutex */
	mutex_unlock(&aew_dev_configptr->read_blocked);
	return result;
}

/* This function will handle interrupt generated by H3A Engine */
static irqreturn_t aew_isr(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	/* Busy AF Bit */
	unsigned int busyaew;

	/* Temporary Buffer for Swapping */
	void *buffer_temp;

	/* Get the value of PCR register */
	busyaew = AEW_GET_PCR;

	/* If AEW engine is not enabled, interrupt is not for AEW */
	if (((busyaew & 0x10000) >> 16) == 0)
		return IRQ_RETVAL(IRQ_NONE);

	/*
	 * Interrupt is generated by AEW, so Service the Interrupt
	 * Swap current buffer and old buffer
	 */
	if (aew_dev_configptr) {
		buffer_temp = aew_dev_configptr->buff_curr;
		aew_dev_configptr->buff_curr = aew_dev_configptr->buff_old;
		aew_dev_configptr->buff_old = buffer_temp;

		/* Set the AEWBUFSTAT Register to current buffer Address */
		aew_set_address((unsigned
			long)(virt_to_phys(aew_dev_configptr->buff_curr)));

		/*
		 * Set buffer filled flag to indicate
		 * statistics are available
		 */
		aew_dev_configptr->buffer_filled = 1;

		/* queue the event with v4l2 */
		aew_queue_event(sd);

		return IRQ_RETVAL(IRQ_HANDLED);
	}
	return IRQ_RETVAL(IRQ_NONE);
}


int dm355_aew_set_stream(struct v4l2_subdev *sd, int enable)
{
	int result = 0;

	if (enable) {
		/* start capture */
		/* Enable AEW Engine if Hardware set up is done */
		if (aew_dev_configptr->aew_config == H3A_AEW_CONFIG_NOT_DONE) {
			dev_err(aewdev,
				"Error : AEW Hardware is not configured.\n");
			result = -EINVAL;
		} else {
			result = request_irq(4, aew_isr, IRQF_SHARED,
					     "dm355_h3a_aew",
					     (void *)sd);
			if (result != 0)
				return result;

			/* Enable AF Engine */
			aew_engine_setup(1);
		}
	} else {
		/* stop capture */
		free_irq(4, sd);
		/* Disable AEW Engine */
		aew_engine_setup(0);
	}

	return result;
}
EXPORT_SYMBOL(dm355_aew_set_stream);

/* Function to register the AF character device driver */
int dm355_aew_init(struct platform_device *pdev)
{
	unsigned int vpssclk;


	/* Allocate memory for device structure and initialize it with 0 */
	aew_dev_configptr =
	    (struct aew_device *)kmalloc(sizeof(struct aew_device),
					 GFP_KERNEL);
	if (!aew_dev_configptr)
		return -ENOMEM;

	/* AEW_SELINT(interrupt_no) */
	AEW_SETGAMMAWD;
	vpssclk = AEW_GETCLKCTRL;
	vpssclk |= (1 << 4);
	AEW_SETCLKCTRL(vpssclk);
	/* Initialize device structure */
	memset(aew_dev_configptr, 0, sizeof(struct aew_device));

	aew_dev_configptr->in_use = AEW_NOT_IN_USE;
	aew_dev_configptr->buffer_filled = 0;

	aewdev = &pdev->dev;

	return 0;
}
EXPORT_SYMBOL(dm355_aew_init);

void  dm355_aew_cleanup(void)
{
	/* Device is in use */
	if (aew_dev_configptr->in_use == AEW_IN_USE)
		return;

	kfree(aew_dev_configptr);
	aew_dev_configptr = NULL;

}
EXPORT_SYMBOL(dm355_aew_cleanup);
