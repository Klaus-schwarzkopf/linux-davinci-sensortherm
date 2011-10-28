/*
 * Copyright (C) 2007 Ridgerun (http://www.ridgerun.com)
 *
 * Author: Natanel Castro <natanael.castro@ridgerun.com>
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
 *
 * Based on logicpd_encoder.c
 */

/* Kernel Specific header files */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <media/davinci/vid_encoder_if.h>
#include <media/davinci/dm365_generic_prgb_encoder.h>

/* Function prototypes */

static __init int gen_prgb_probe(struct platform_device *pdev);
static int gen_prgb_remove(struct platform_device *pdev);

static int gen_prgb_encoder_initialize(struct vid_encoder_device *enc, int flag);
static int gen_prgb_encoder_deinitialize(struct vid_encoder_device *enc);

static int gen_prgb_driver_init(void);
static void gen_prgb_driver_cleanup(void);


static int gen_prgb_encoder_setmode(struct vid_enc_mode_info *mode_info,
				   struct vid_encoder_device *enc);
static int gen_prgb_encoder_getmode(struct vid_enc_mode_info *mode_info,
				   struct vid_encoder_device *enc);

static int gen_prgb_encoder_setoutput(char *output,
				     struct vid_encoder_device *enc);
static int gen_prgb_encoder_getoutput(char *output,
				     struct vid_encoder_device *enc);

static int gen_prgb_encoder_enumoutput(int index,
				      char *output,
				      struct vid_encoder_device *enc);

static struct gen_prgb_encoder_config gen_prgb_encoder_configuration = {
	.no_of_outputs = GEN_PRGB_ENCODER_MAX_NO_OUTPUTS,
	.output[0] = {
		      .output_name = VID_ENC_OUTPUT_LCD,
		      .no_of_standard = GEN_PRGB_ENCODER_NUM_STD,
		      .standards[0] = {	/* This is programmed by the driver when
								.probe function is called */
				       .name = VID_ENC_STD_PRGB_DEFAULT,
				       .std = 1,
				       .if_type = VID_ENC_IF_PRGB,
				       .interlaced = 0,
				       .xres = 0,
				       .yres = 0,
				       .fps = {0, 0},
				       .left_margin = 0,
				       .right_margin = 0,
				       .upper_margin = 0,
				       .lower_margin = 0,
				       .hsync_len = 0,
				       .vsync_len = 0,
				       .flags = 0}, /* hsync -ve, vsync -ve */
		      },
};

static struct gen_prgb_encoder_channel gen_prgb_encoder_channel_info = {
	.params.outindex = 0,
	.params.mode = VID_ENC_STD_PRGB_DEFAULT,
	.enc_device = NULL
};

static struct vid_enc_output_ops outputs_ops = {
	.count = GEN_PRGB_ENCODER_MAX_NO_OUTPUTS,
	.enumoutput = gen_prgb_encoder_enumoutput,
	.setoutput = gen_prgb_encoder_setoutput,
	.getoutput = gen_prgb_encoder_getoutput
};

static struct vid_enc_mode_ops modes_ops = {
	.setmode = gen_prgb_encoder_setmode,
	.getmode = gen_prgb_encoder_getmode,
};

/* struct for encoder registration */
static struct vid_encoder_device gen_prgb_encoder_dev = {
	.name = "GEN_PRGB_ENCODER",
	.capabilities = 0,
	.initialize = gen_prgb_encoder_initialize,
	.mode_ops = &modes_ops,
	.ctrl_ops = NULL,
	.output_ops = &outputs_ops,
	.params_ops = NULL,
	.misc_ops = NULL,
	.deinitialize = gen_prgb_encoder_deinitialize,
};

/* struct for driver registration */
static struct platform_driver gen_prgb_driver = {
	.driver = {
		.name = PRGB_ENCODER_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = gen_prgb_probe,
	.remove = __devexit_p(gen_prgb_remove),
};

/* This function is called by the encoder manager to initialize gen_prgb encoder driver.
 */
static int gen_prgb_encoder_initialize(struct vid_encoder_device *enc, int flag)
{
	int err = 0, outindex;
	char *std, *output;
	if (NULL == enc) {
		printk(KERN_ERR "enc:NULL Pointer\n");
		return -EINVAL;
	}
	gen_prgb_encoder_channel_info.enc_device = (struct encoder_device *)enc;

	/* call set standard */
	std = gen_prgb_encoder_channel_info.params.mode;
	outindex = gen_prgb_encoder_channel_info.params.outindex;
	output = gen_prgb_encoder_configuration.output[outindex].output_name;
	err |= gen_prgb_encoder_setoutput(output, enc);
	if (err < 0) {
		err = -EINVAL;
		printk(KERN_ERR "Error occured in setoutput\n");
		gen_prgb_encoder_deinitialize(enc);
		return err;
	}
	printk(KERN_DEBUG "General PRGB Encoder initialized\n");
	return err;
}

/* Function to de-initialize the encoder */
static int gen_prgb_encoder_deinitialize(struct vid_encoder_device *enc)
{
	if (NULL == enc) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	gen_prgb_encoder_channel_info.enc_device = NULL;
	printk(KERN_DEBUG "General PRGB Encoder de-initialized\n");
	return 0;
}

/* Following function is used to set the mode*/
static int gen_prgb_encoder_setmode(struct vid_enc_mode_info *mode_info,
				   struct vid_encoder_device *enc)
{
	int err = 0, outindex, i;
	char *mode;
	struct vid_enc_mode_info *my_mode_info = NULL;

	if ((NULL == enc) || (NULL == mode_info)) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}

	if (NULL == (mode = mode_info->name)) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	printk(KERN_DEBUG "Start of gen_prgb_encoder_setmode..\n");
	outindex = gen_prgb_encoder_channel_info.params.outindex;

	if (mode_info->std) {
		char *mymode = NULL;
		/* This is a standard mode */
		for (i = 0;
		     i <
		     gen_prgb_encoder_configuration.output[outindex].
		     no_of_standard; i++) {
			if (!strcmp
			    (gen_prgb_encoder_configuration.output[outindex].
			     standards[i].name, mode)) {
				mymode =
				    gen_prgb_encoder_configuration.
				    output[outindex].standards[i].name;
				break;
			}
		}
		if ((i ==
		     gen_prgb_encoder_configuration.output[outindex].
		     no_of_standard) || (NULL == mymode)) {
			printk(KERN_ERR "Invalid id...\n");
			return -EINVAL;
		}
		/* Store the standard in global object of gen_prgb_encoder */
		gen_prgb_encoder_channel_info.params.mode = mymode;
		return 0;
	}
	printk(KERN_DEBUG "</gen_prgb_encoder_setmode>\n");
	return err;
}

/* Following function is used to get currently selected mode.*/
static int gen_prgb_encoder_getmode(struct vid_enc_mode_info *mode_info,
				   struct vid_encoder_device *enc)
{
	int err = 0, i, outindex;
	if ((NULL == enc) || (NULL == mode_info)) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	printk(KERN_DEBUG "<gen_prgb_encoder_getmode>\n");
	outindex = gen_prgb_encoder_channel_info.params.outindex;
	for (i = 0; i < GEN_PRGB_ENCODER_NUM_STD; i++) {
		if (!strcmp(gen_prgb_encoder_channel_info.params.mode,
			    gen_prgb_encoder_configuration.output[outindex].
			    standards[i].name)) {
			memcpy(mode_info,
			       &gen_prgb_encoder_configuration.output[outindex].
			       standards[i], sizeof(struct vid_enc_mode_info));
			break;
		}
	}
	if (i == GEN_PRGB_ENCODER_NUM_STD) {
		printk(KERN_ERR "Wiered. No mode info\n");
		return -EINVAL;
	}
	printk(KERN_DEBUG "</gen_prgb_encoder_getmode>\n");
	return err;
}

/* For General PRGB, we have several outputs, we
   always set this to this at init
*/
static int gen_prgb_encoder_setoutput(char *output,
				     struct vid_encoder_device *enc)
{
	int err = 0;
	struct vid_enc_mode_info *my_mode_info;
	printk(KERN_DEBUG "<gen_prgb_encoder_setoutput>\n");
	if (NULL == enc) {
		printk(KERN_ERR "enc:NULL Pointer\n");
		return -EINVAL;
	}

	/* check for null pointer */
	if (output == NULL) {
		printk(KERN_ERR "output: NULL Pointer.\n");
		return -EINVAL;
	}

	/* Just check if the default output match with this output name */
	if (strcmp(gen_prgb_encoder_configuration.output[0].output_name, output)) {
		printk(KERN_ERR "no matching output found.\n");
		return -EINVAL;
	}
	gen_prgb_encoder_channel_info.params.mode
	    = gen_prgb_encoder_configuration.output[0].standards[0].name;

	my_mode_info = &gen_prgb_encoder_configuration.output[0].standards[0];
	err |= gen_prgb_encoder_setmode(my_mode_info, enc);
	if (err < 0) {
		printk(KERN_ERR "Error in setting default mode\n");
		return err;
	}
	printk(KERN_DEBUG "</gen_prgb_encoder_setoutput>\n");
	return err;
}

/* Following function is used to get output name of current output.*/
static int gen_prgb_encoder_getoutput(char *output,
				     struct vid_encoder_device *enc)
{
	int err = 0, index, len;
	if (NULL == enc) {
		printk(KERN_ERR "enc:NULL Pointer\n");
		return -EINVAL;
	}
	printk(KERN_DEBUG "<gen_prgb_encoder_getoutput>\n");
	/* check for null pointer */
	if (output == NULL) {
		printk(KERN_ERR "output:NULL Pointer.\n");
		return -EINVAL;
	}
	index = gen_prgb_encoder_channel_info.params.outindex;
	len = strlen(gen_prgb_encoder_configuration.output[index].output_name);
	if (len > (VID_ENC_NAME_MAX_CHARS - 1))
		len = VID_ENC_NAME_MAX_CHARS - 1;
	strncpy(output, gen_prgb_encoder_configuration.output[index].output_name,
		len);
	output[len] = '\0';
	printk(KERN_DEBUG "</gen_prgb_encoder_getoutput>\n");
	return err;
}

/* Following function is used to enumerate outputs supported by the driver.
   It fills in information in the output. */
static int gen_prgb_encoder_enumoutput(int index, char *output,
				      struct vid_encoder_device *enc)
{
	int err = 0;

	printk(KERN_DEBUG "<gen_prgb_encoder_enumoutput>\n");
	if (NULL == enc) {
		printk(KERN_ERR "enc:NULL Pointer.\n");
		return -EINVAL;
	}
	/* check for null pointer */
	if (output == NULL) {
		printk(KERN_ERR "output:NULL Pointer.\n");
		return -EINVAL;
	}
	/* Only one output is available */
	if (index >= gen_prgb_encoder_configuration.no_of_outputs) {
		return -EINVAL;
	}
	strncpy(output,
		gen_prgb_encoder_configuration.output[index].output_name,
		VID_ENC_NAME_MAX_CHARS);
	printk(KERN_DEBUG "</gen_prgb_encoder_enumoutput>\n");
	return err;
}


/* Encoder registration with the encoder manager */
static int __init gen_prgb_probe(struct platform_device *pdev)
{
	int err = 0;

	struct davinci_gen_prgb_pdata *gen_prgb_dev = (struct davinci_gen_prgb_pdata *) (*pdev).dev.platform_data;

	printk(KERN_NOTICE "General PRGB probe function\n");

	/* Setting up prgb parameters*/

	gen_prgb_encoder_configuration.output[0].standards[0].xres 			= gen_prgb_dev->xres;
	gen_prgb_encoder_configuration.output[0].standards[0].yres 			= gen_prgb_dev->yres;
	gen_prgb_encoder_configuration.output[0].standards[0].fps 			= gen_prgb_dev->fps;
	gen_prgb_encoder_configuration.output[0].standards[0].left_margin 	= gen_prgb_dev->left_margin;
	gen_prgb_encoder_configuration.output[0].standards[0].right_margin 	= gen_prgb_dev->right_margin;
	gen_prgb_encoder_configuration.output[0].standards[0].upper_margin 	= gen_prgb_dev->upper_margin;
	gen_prgb_encoder_configuration.output[0].standards[0].lower_margin 	= gen_prgb_dev->lower_margin;
	gen_prgb_encoder_configuration.output[0].standards[0].hsync_len 	= gen_prgb_dev->hsync_len;
	gen_prgb_encoder_configuration.output[0].standards[0].vsync_len 	= gen_prgb_dev->vsync_len;
	gen_prgb_encoder_configuration.output[0].standards[0].flags		 	= gen_prgb_dev->flags;

	err = vid_enc_register_encoder(&gen_prgb_encoder_dev);

	return err;
}

/* This function used to un-register the General PRGB driver */
static int gen_prgb_remove(struct platform_device *pdev)
{
	vid_enc_unregister_encoder(&gen_prgb_encoder_dev);

	return 0;
}

/* This function used to un-initialize the General PRGB driver */
static int gen_prgb_driver_init(void)
{
	int err = 0;

	printk(KERN_NOTICE "gen_prgb_driver_init\n");
	/* Register driver to the kernel */
	err = platform_driver_register(&gen_prgb_driver);

	return err;
}

/* This function used to un-initialize the General PRGB driver */
static void gen_prgb_driver_cleanup(void)
{
	platform_driver_unregister(&gen_prgb_driver);
}

subsys_initcall(gen_prgb_driver_init);
module_exit(gen_prgb_driver_cleanup);

MODULE_LICENSE("GPL");
