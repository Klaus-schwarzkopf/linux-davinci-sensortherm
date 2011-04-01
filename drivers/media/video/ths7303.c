/*
 * ths7303- THS7303 Video Amplifier driver
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#define THS7303_CHANNEL_1 (1)
#define THS7303_CHANNEL_2 (2)
#define THS7303_CHANNEL_3 (3)

enum ths7303_filter_mode {
	THS7303_FILTER_MODE_480I_576I,
	THS7303_FILTER_MODE_480P_576P,
	THS7303_FILTER_MODE_720P_1080I,
	THS7303_FILTER_MODE_1080P,
	THS7303_FILTER_MODE_DISABLE
};

MODULE_DESCRIPTION("TI THS7303 video amplifier driver");
MODULE_AUTHOR("Chaithrika U S");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");

/* following function is used to set ths7303 */
int ths7303_setval(struct v4l2_subdev *sd, enum ths7303_filter_mode mode)
{
	u8 val = 0, input_bias_luma = 3, input_bias_chroma = 3, temp;
	int err = 0, disable = 0;

	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (client == NULL)
		return err;


	switch (mode) {
	case THS7303_FILTER_MODE_1080P:
		val = (3 << 6);
		val |= (3 << 3);
		break;
	case THS7303_FILTER_MODE_720P_1080I:
		val = (2 << 6);
		val |= (2 << 3);
		break;
	case THS7303_FILTER_MODE_480P_576P:
		val = (1 << 6);
		val |= (1 << 3);
		break;
	case THS7303_FILTER_MODE_480I_576I:
		break;
	case THS7303_FILTER_MODE_DISABLE:
		printk(KERN_INFO "mode disabled\n");
		/* disable all channels */
		disable = 1;
	default:
		/* disable all channels */
		disable = 1;
	}
	/* Setup channel 2 - Luma - Green */
	temp = val;
	if (!disable)
		val |= input_bias_luma;
	err = i2c_smbus_write_byte_data(client, THS7303_CHANNEL_2, val);
	if (err)
		goto out;

	/* setup two chroma channels */
	if (!disable)
		temp |= input_bias_chroma;

	err = i2c_smbus_write_byte_data(client, THS7303_CHANNEL_1, temp);
	if (err)
		goto out;

	err = i2c_smbus_write_byte_data(client, THS7303_CHANNEL_3, temp);
	if (err)
		goto out;
out:
	printk(KERN_INFO "write byte data failed\n");
	return err;
}

static int ths7303_s_std_output(struct v4l2_subdev *sd, v4l2_std_id norm)
{
	if (norm & (V4L2_STD_ALL & ~V4L2_STD_SECAM))
		return ths7303_setval(sd, THS7303_FILTER_MODE_480I_576I);
	else
		return ths7303_setval(sd, THS7303_FILTER_MODE_DISABLE);
}

/* for setting filter for HD output */
static int ths7303_s_dv_preset(struct v4l2_subdev *sd,
			       struct v4l2_dv_preset *preset)
{
	int res = 0;

	switch (preset->preset) {
	case V4L2_DV_1080P60:
		res = ths7303_setval(sd, THS7303_FILTER_MODE_1080P);
		break;
	case V4L2_DV_720P60:
	case V4L2_DV_1080I30:
		res = ths7303_setval(sd, THS7303_FILTER_MODE_720P_1080I);
		break;
	case V4L2_DV_480P59_94:
	case V4L2_DV_576P50:
		res = ths7303_setval(sd, THS7303_FILTER_MODE_480P_576P);
		break;
	default:
		/* disable all channels */
		res = ths7303_setval(sd, THS7303_FILTER_MODE_DISABLE);
	}

	return res;
}

static int ths7303_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_THS7303, 0);
}

static const struct v4l2_subdev_video_ops ths7303_video_ops = {
	.s_std_output	= ths7303_s_std_output,
	.s_dv_preset    = ths7303_s_dv_preset,
};

static const struct v4l2_subdev_core_ops ths7303_core_ops = {
	.g_chip_ident = ths7303_g_chip_ident,
};

static const struct v4l2_subdev_ops ths7303_ops = {
	.core	= &ths7303_core_ops,
	.video 	= &ths7303_video_ops,
};

static int ths7303_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	sd = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (sd == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(sd, client, &ths7303_ops);

	return ths7303_s_std_output(sd, V4L2_STD_NTSC);
}

static int ths7303_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(sd);

	return 0;
}

static const struct i2c_device_id ths7303_id[] = {
	{"ths7303", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ths7303_id);

static struct i2c_driver ths7303_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ths7303",
	},
	.probe		= ths7303_probe,
	.remove		= ths7303_remove,
	.id_table	= ths7303_id,
};

static int __init ths7303_init(void)
{
	return i2c_add_driver(&ths7303_driver);
}

static void __exit ths7303_exit(void)
{
	i2c_del_driver(&ths7303_driver);
}

module_init(ths7303_init);
module_exit(ths7303_exit);

