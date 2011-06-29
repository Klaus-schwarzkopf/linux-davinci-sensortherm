/*
 * Driver for MT9V126 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2010, Klaus Schwarzkopf, <schwarzkopf@sensortherm.de>
 *
 * Heavily based on MT9T031 driver from Guennadi Liakhovetski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/log2.h>

#include <media/mt9v126.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>



#include "rw_i2c_reg.h"
#include "mt9v126_reg_init.h"

/* mt9v126 i2c address 0x48
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */

#define TEST_DATA_RED 		0x3072
#define TEST_DATA_GREENR 	0x3074
#define TEST_DATA_BLUE 		0x3076
#define TEST_DATA_GREENB	0x3078


#define TEST_PATTERN_MODE 	0

#define FM_PROC_BAYER_FIRST_COLOR_GREEN1 	(0 << 9)
#define FM_PROC_BAYER_FIRST_COLOR_RED		(1 << 9)
#define FM_PROC_BAYER_FIRST_COLOR_BLUE		(2 << 9)
#define FM_PROC_BAYER_FIRST_COLOR_GREEN2	(3 << 9)	





#define FM_PROC_BAYER_FIRST_COLOR FM_PROC_BAYER_FIRST_COLOR_GREEN1

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static const struct v4l2_fmtdesc mt9v126_formats[] = {
/*		{
				.index = 0,
				.type =	V4L2_BUF_TYPE_VIDEO_CAPTURE,
				.description = "Bayer (sRGB) 10 bit",
				.pixelformat = V4L2_PIX_FMT_SGRBG10,
		},*/
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
};
static const unsigned int mt9v126_num_formats = ARRAY_SIZE(mt9v126_formats);


void get_camera_info(struct v4l2_subdev *sd) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk( KERN_DEBUG "Monitor Variables 63\n" );
	reg_read(client, 0x8000);
	reg_read(client, 0x8002);
	reg_read(client, 0x8004);
	reg_read(client, 0x8006);
	printk( KERN_DEBUG "frame_status 34\n" );
	reg_read(client, 0x303c);
	printk( KERN_DEBUG "read_mode 34\n" );
	reg_read(client, 0x3040);
	printk( KERN_DEBUG "test_pattern_mode 36\n" );
	reg_read(client, 0x3070);
	printk( KERN_DEBUG "output_format_configuration 42\n" );
	reg_read(client, 0x332e);
	printk( KERN_DEBUG "k22b_chip_id 52\n" );
	reg_read(client, 0x0000);
	printk( KERN_DEBUG "pll_control 53\n" );
	reg_read(client, 0x0014);
	printk( KERN_DEBUG "reset_and_misc_control 53\n" );
	reg_read(client, 0x001a);
	printk( KERN_DEBUG "parallel_bus_ctrl 58\n" );
	reg_read(client, 0x3c00);
	printk( KERN_DEBUG "frame count 43\n" );
	reg_read(client, 0x3334);

}



enum {
	BAYER_8_2_PROGRESSIVE ,
	YCBCR_RGB_PROGRESSIVE_RGB565,
	YCBCR_RGB_PROGRESSIVE_BAYER,
	YCBCR_RGB_PROGRESSIVE_YCBCR,
	DEWARPED_YCBCR_PROGRESSIVE,
	DEWARPED_BT656_INTERLACED,
	DEWARPED_OVERLAY_BT656_INTERLACED,
	TEST_PATTERN_BT656_INTERLACED,
};

#define IF_TYPE 	DEWARPED_OVERLAY_BT656_INTERLACED

void set_if_type(struct i2c_client *client, int if_type) {

	int bayer = 0x0040 | FM_PROC_BAYER_FIRST_COLOR;

	printk( KERN_INFO "bayer 0x%x\n", bayer );
	
	switch(IF_TYPE)
	{
		case BAYER_8_2_PROGRESSIVE:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			reg_write(client, 0xfc00, 0x7000);
			reg_write(client, 0x0040, 0x8801);
			
			
			
			reg_write(client, 0x332e, bayer);
			
			//reg_write(client, 0x332e, 0x0040);
			reg_write(client, 0x3070, 0x0000);

			//648x488 -> 640x480 (VGA)
			//FIXME: center the picture
			//reg_write(client, 0x3002, 0x0014); //y_addr_start
			//reg_write(client, 0x3004, 0x0018); //x_addr_start
			break;

		case YCBCR_RGB_PROGRESSIVE_RGB565:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0003);
			reg_write(client, 0x0040, 0x8800);
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			reg_write(client, 0xfc00, 0x3000);
			reg_write(client, 0x0040, 0x8801);

			reg_write(client, 0x332e, 0x0020);
			reg_write(client, 0x332e, 0x0020);
			reg_write(client, 0x3070, 0x0000);
			break;
		case YCBCR_RGB_PROGRESSIVE_BAYER:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0003);
			reg_write(client, 0x0040, 0x8800);
			
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			
			reg_write(client, 0xfc00, 0x3000);
			reg_write(client, 0x0040, 0x8801);

			reg_write(client, 0x332e, 0x0040);
			reg_write(client, 0x332e, 0x0040);
			reg_write(client, 0x3070, 0x0000);
			break;
		case YCBCR_RGB_PROGRESSIVE_YCBCR:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0003);
			reg_write(client, 0x0040, 0x8800);
			
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			
			reg_write(client, 0xfc00, 0x3000);
			reg_write(client, 0x0040, 0x8801);
			
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x3070, 0x0000);
			break;
		case DEWARPED_YCBCR_PROGRESSIVE:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0103);
			reg_write(client, 0x0040, 0x8800);
			
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			reg_write(client, 0xfc00, 0x4000);
			reg_write(client, 0x0040, 0x8801);
			
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x3070, 0x0000);
			break;
		case DEWARPED_BT656_INTERLACED:
			//done
			reg_write(client, 0x098e, 0x7c00);
			
			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0103);
			reg_write(client, 0x0040, 0x8800);

			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			
			reg_write(client, 0xfc00, 0x2000);
			reg_write(client, 0x0040, 0x8801);
			
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x3070, 0x0000);
			break;
		case DEWARPED_OVERLAY_BT656_INTERLACED:
			//done
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0000);
			reg_write(client, 0xfc04, 0x0103);
			reg_write(client, 0x0040, 0x8800);
			
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			
			reg_write(client, 0xfc00, 0x1000);
			reg_write(client, 0x0040, 0x8801);
			
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x332e, 0x0000);
			reg_write(client, 0x3070, 0x0000);
			break;
		case TEST_PATTERN_BT656_INTERLACED:
			//do not work
			reg_write(client, 0x098e, 0x7c00);

			reg_write(client, 0xfc00, 0x0000);
			reg_write(client, 0xfc02, 0x0100);
			reg_write(client, 0xfc04, 0x0103);			
			reg_write(client, 0x0040, 0x8800);
			
			reg_write(client, 0xfc00, 0x0100);
			reg_write(client, 0x0040, 0x8802);
			
			reg_write(client, 0xfc00, 0x5000);
			reg_write(client, 0x0040, 0x8801);

			reg_write(client, 0x3070, 0x0001);
			break;

			
	}
	
	//Test pattern mode
	reg_write(client, 0x3070, TEST_PATTERN_MODE);
	
	switch(TEST_PATTERN_MODE)
	{
		case 0: //Normal operation
			//nothing to do 
		break;
		case 1: // Solid color test pattern
			//only Red
			reg_write(client, TEST_DATA_RED, 0x03FF);
			reg_write(client, TEST_DATA_GREENR, 0x0000);
			reg_write(client, TEST_DATA_BLUE, 0x0000);
			reg_write(client, TEST_DATA_GREENB, 0x0000);
		break;
		case 2: // 100% color bar test pattern (color columns)
			//nothing to do 
		break;
		
		default:
			
			//nothing to do 
		break;
		
	}



}

//TODO
static const struct v4l2_queryctrl mt9v126_controls[] = {
                {
				.id = V4L2_CID_VFLIP,
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.name =	"Flip Vertically",
				.minimum = 0,
				.maximum = 1,
				.step = 1,
				.default_value = 0,
		},
		{
				.id = V4L2_CID_HFLIP,
				.type = V4L2_CTRL_TYPE_BOOLEAN,
				.name = "Flip Horizontally",
				.minimum = 0,
				.maximum = 1,
				.step = 1,
				.default_value = 0,
		},

};
static const unsigned int mt9v126_num_controls = ARRAY_SIZE(mt9v126_controls);

struct mt9v126 {
	struct v4l2_subdev sd;
	int model; /* V4L2_IDENT_MT9V126* codes from v4l2-chip-ident.h */
	unsigned char autoexposure;
	u16 xskip;
	u16 yskip;
	u32 width;
	u32 height;
	unsigned short x_min; /* Camera capabilities */
	unsigned short y_min;
	unsigned short x_current; /* Current window location */
	unsigned short y_current;
	unsigned short width_min;
	unsigned short width_max;
	unsigned short height_min;
	unsigned short height_max;
	unsigned short y_skip_top; /* Lines to skip at the top */
	unsigned short gain;
	unsigned short exposure;
	/*mc related members */
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
};

static inline struct mt9v126 *to_mt9v126(struct v4l2_subdev *sd) {
	return container_of(sd, struct mt9v126, sd);
}

//TODO
static int mt9v126_init(struct v4l2_subdev *sd, u32 val) {

	int ret =1;
	//struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk( KERN_DEBUG "module init\n" );
	/* Disable chip output, synchronous option update */
//	ret = mt9v126_reg_write(client, MT9V126_RESET, 1);
//	if (ret >= 0)
//		ret = mt9v126_reg_write(client, MT9V126_RESET, 0);
//	if (ret >= 0)
//		ret = mt9v126_reg_clear(client, MT9V126_OUTPUT_CONTROL, 2);

//	ret = reg_set(client, MT9V126_REG_RESET_REGISTER, MT9V126_VALUE_RESET_REGISTER_RESET);
//	ret = reg_read(client, MT9V126_REG_OUTPUT_FORMAT_CONFIGURATION);
//#ifdef TEST_PATTERN
//	ret = reg_write(client, MT9V126_REG_TEST_PATTERN_MODE, MT9V126_VALUE_TEST_PATTERN_MODE_COLOR_BAR);
//#endif
	return ret >= 0 ? 0 : -EIO;
}

static int mt9v126_s_stream(struct v4l2_subdev *sd, int enable) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk( KERN_DEBUG "s_stream\n" );


	/* Switch to master "normal" mode */

	if (enable) {
		if (reg_set(client, MT9V126_REG_RESET_REGISTER, 0x0004) < 0)
			return -EIO;
	} else {
		/* Switch to master "" mode */
		if (reg_clear(client, MT9V126_REG_RESET_REGISTER, 0x0004) < 0)
			return -EIO;
	}
	
	//get_camera_info(sd);
	return 0;
}


const struct v4l2_queryctrl *mt9v126_find_qctrl(u32 id) {

	int i;

	printk( KERN_DEBUG "find_qctrl\n" );
	for (i = 0; i < mt9v126_num_controls; i++) {
		if (mt9v126_controls[i].id == id)
			return &mt9v126_controls[i];
	}
	return NULL;
}


static int mt9v126_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *f) {

	//struct mt9v126 *mt9v126 = to_mt9v126(sd);
	int ret =0;
	//u16 xskip, yskip;


	printk( KERN_DEBUG "module_set_fmt\n" );

	return ret;
}

static int mt9v126_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *f) {

#if 0
	struct v4l2_pix_format *pix = &f->fmt.pix;

	printk( KERN_DEBUG "modul try_fmt\n" );
	if (pix->height < MT9V126_MIN_HEIGHT)
		pix->height = MT9V126_MIN_HEIGHT;
	if (pix->height > MT9V126_MAX_HEIGHT)
		pix->height = MT9V126_MAX_HEIGHT;
	if (pix->width < MT9V126_MIN_WIDTH)
		pix->width = MT9V126_MIN_WIDTH;
	if (pix->width > MT9V126_MAX_WIDTH)
		pix->width = MT9V126_MAX_WIDTH;

	pix->width &= ~0x01; /* has to be even */
	pix->height &= ~0x01; /* has to be even */
#endif
	return 0;
}

/* enumerates mbus_fmt supported */
static int mt9v126_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	u32 pad = code->pad;
	u32 index = code->index;

	memset(code, 0, sizeof(*code));
	code->index = index;
	code->pad = pad;

	if (index != 0)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_YUYV8_2X8;
	return 0;
}

static int mt9v126_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{

	if (fse->index > 0)
		return -EINVAL;

	fse->min_width = 640;
	fse->min_height = 480;
	fse->max_width = 640;
	fse->max_height = 480;

	/* values should be even */
	fse->min_width &= ~0x01;
	fse->min_height &= ~0x01;
	fse->max_width &= ~0x01;
	fse->max_height &= ~0x01;

	return 0;
}

static int mt9v126_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	__u32 which	= fmt->which;
	struct mt9v126 *mt9v126 = to_mt9v126(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE)
		fmt->format = mt9v126->format;
	else {
		fmt->format.code = V4L2_MBUS_FMT_YUYV8_2X8;
		fmt->format.width = clamp_t(u32, fmt->format.width,
					    640,
					    640);
		fmt->format.height = clamp_t(u32,
					     fmt->format.height,
					     480,
					     480);
		fmt->format.field = V4L2_FIELD_NONE;
	}

	return 0;
}

static int mt9v126_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct mt9v126 *mt9v126 = to_mt9v126(sd);
	u16 xskip, yskip;
	int ret =0;

	struct v4l2_rect rect = {
		.left	= mt9v126->x_current,
		.top	= mt9v126->y_current,
		.width	= fmt->format.width,
		.height	= fmt->format.height,
	};

	if (fmt->format.code != V4L2_MBUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	/* Is this more optimal than just a division? */
	for (xskip = 8; xskip > 1; xskip--)
		if (rect.width * xskip <= 640)
			break;

	for (yskip = 8; yskip > 1; yskip--)
		if (rect.height * yskip <= 480)
			break;



	mt9v126->xskip = xskip;
	mt9v126->yskip = yskip;

	mt9v126->format = fmt->format;
	return ret;
}

static int mt9v126_get_chip_id(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *id) {

	struct mt9v126 *mt9v126 = to_mt9v126(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk( KERN_DEBUG "get_chip_id\n" );
	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident = mt9v126->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9v126_get_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg) {

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct mt9v126 *mt9v126 = to_mt9v126(sd);

	printk( KERN_DEBUG "get_register\n" );
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int mt9v126_set_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg) {

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct mt9v126 *mt9v126 = to_mt9v126(sd);

	printk( KERN_DEBUG "set_register\n" );
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}

#endif
#if 0
static int mt9v126_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct v4l2_fmtdesc *ofmt;

	if (fmt->index >= ARRAY_SIZE(mt9v126_formats))
		return -EINVAL;

	ofmt = mt9v126_formats + fmt->index;
	fmt->flags = ofmt->flags;
	fmt->type = ofmt->type;
	strcpy(fmt->description, ofmt->description);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}
#endif

static int mt9v126_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9v126_set_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9v126_queryctrl(struct v4l2_subdev *, struct v4l2_queryctrl *);

static const struct v4l2_subdev_core_ops mt9v126_core_ops = { 
	.g_chip_ident = mt9v126_get_chip_id, 
	.init = mt9v126_init, 
	.queryctrl = mt9v126_queryctrl, 
	.g_ctrl = mt9v126_get_control, 
	.s_ctrl = mt9v126_set_control,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = mt9v126_get_register, 
	.s_register = mt9v126_set_register,
#endif
};

static const struct v4l2_subdev_video_ops mt9v126_video_ops = { 
	.s_mbus_fmt = mt9v126_s_fmt, 
	.try_mbus_fmt = mt9v126_try_fmt, 
	.s_stream = mt9v126_s_stream, 
	//.enum_fmt = mt9v126_enum_fmt,
};

static const struct v4l2_subdev_pad_ops mt9v126_pad_ops = {
	.enum_mbus_code = mt9v126_enum_mbus_code,
	.enum_frame_size = mt9v126_enum_frame_size,
	.get_fmt = mt9v126_get_pad_format,
	.set_fmt = mt9v126_set_pad_format,
};

static const struct v4l2_subdev_ops mt9v126_ops = { 
	.core = &mt9v126_core_ops,
	.video = &mt9v126_video_ops, 
	.pad = &mt9v126_pad_ops,
};

static int mt9v126_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qctrl) {

	const struct v4l2_queryctrl *temp_qctrl;

	printk( KERN_DEBUG "queryctrl\n" );
	temp_qctrl = mt9v126_find_qctrl(qctrl->id);
	if (!temp_qctrl) {
		v4l2_err(sd, "control id %d not supported", qctrl->id);
		return -EINVAL;
	}
	memcpy(qctrl, temp_qctrl, sizeof(*qctrl));
	return 0;
}
//TODO:Test 
static int mt9v126_get_control(struct v4l2_subdev *sd,
		struct v4l2_control *ctrl) {

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9v126 *mt9v126 = to_mt9v126(sd);
	int data;

	printk( KERN_DEBUG "get_control\n" );
	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		data = reg_read(client, MT9V126_REG_READ_MODE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & MT9V126_VALUE_READ_MODE_VERT_FLIP);
		break;
	case V4L2_CID_HFLIP:
		data = reg_read(client, MT9V126_REG_READ_MODE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & MT9V126_VALUE_READ_MODE_HORIZ_MIRROR);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ctrl->value = mt9v126->autoexposure;
		break;
	}
	return 0;
}
//TODO
static int mt9v126_set_control(struct v4l2_subdev *sd,
		struct v4l2_control *ctrl) {

	//struct mt9v126 *mt9v126 = to_mt9v126(sd);
	const struct v4l2_queryctrl *qctrl = NULL;
	int data;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk( KERN_DEBUG "set_control\n" );
	if (NULL == ctrl)
		return -EINVAL;

	qctrl = mt9v126_find_qctrl(ctrl->id);
	if (!qctrl) {
		v4l2_err(sd, "control id %d not supported", ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		if (ctrl->value)
			data = reg_set(client, MT9V126_REG_READ_MODE, MT9V126_VALUE_READ_MODE_VERT_FLIP);
		else
			data = reg_clear(client, MT9V126_REG_READ_MODE, MT9V126_VALUE_READ_MODE_VERT_FLIP);
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->value)
			data = reg_set(client, MT9V126_REG_READ_MODE, MT9V126_VALUE_READ_MODE_HORIZ_MIRROR);
		else
			data = reg_clear(client, MT9V126_REG_READ_MODE, MT9V126_VALUE_READ_MODE_HORIZ_MIRROR);
		if (data < 0)
			return -EIO;
		break;

	}
	return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
//TODO
static int mt9v126_detect(struct i2c_client *client, int *model) {
	s32 data;

	printk( KERN_DEBUG "detect\n" );
	/* Enable the chip */
	//TODO: No register found to enable the chip
	//data = mt9v126_reg_write(client, MT9V126_CHIP_ENABLE, 1);
	//dev_dbg(&client->dev, "write: %d\n", data);

	/* Read out the chip version register */
	data = reg_read(client, MT9V126_CHIP_VERSION);

	switch (data) {
	case 0x2281:
		*model = V4L2_IDENT_MT9V126;
		break;
	default:
		dev_err(&client->dev,
				"No MT9V126 chip detected, register read %x\n", data);
		return -ENODEV;
	}

	dev_info(&client->dev, "Detected a MT9V126 chip ID %x\n", data);

	write_regs(client,reg_list_init);

	set_if_type(client, IF_TYPE);
	return 0;
}
//TODO
static int mt9v126_probe(struct i2c_client *client,
		const struct i2c_device_id *did) {

	struct mt9v126 *mt9v126;
	struct v4l2_subdev *sd;
	int pclk_pol;
	int ret;

	printk( KERN_DEBUG "module probe\n" );
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev,
				"I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "No platform data!!\n");
		return -ENODEV;
	}

	pclk_pol = (int) client->dev.platform_data;

	mt9v126 = kzalloc(sizeof(struct mt9v126), GFP_KERNEL);
	if (!mt9v126)
		return -ENOMEM;

	ret = mt9v126_detect(client, &mt9v126->model);
	if (ret)
		goto clean;

	mt9v126->x_min = MT9V126_COLUMN_SKIP;
	mt9v126->y_min = MT9V126_ROW_SKIP;
	mt9v126->width = MT9V126_DEFAULT_WIDTH;
	mt9v126->height = MT9V126_DEFAULT_WIDTH;
	mt9v126->x_current = mt9v126->x_min;
	mt9v126->y_current = mt9v126->y_min;
	mt9v126->width_min = MT9V126_MIN_WIDTH;
	mt9v126->width_max = MT9V126_MAX_WIDTH;
	mt9v126->height_min = MT9V126_MIN_HEIGHT;
	mt9v126->height_max = MT9V126_MAX_HEIGHT;
	mt9v126->y_skip_top = 10;
	mt9v126->autoexposure = 1;
	mt9v126->xskip = 1;
	mt9v126->yskip = 1;

	/* Register with V4L2 layer as slave device */
	sd = &mt9v126->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9v126_ops);

	mt9v126->pad.flags = MEDIA_PAD_FL_OUTPUT;
	mt9v126->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	mt9v126->sd.entity.flags |= MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&mt9v126->sd.entity, 1, &mt9v126->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "%s sensor driver failed to register !!\n",
			 sd->name);

		goto clean;
	}

	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);
	return 0;

	clean: kfree(mt9v126);
	return ret;
}

static int mt9v126_remove(struct i2c_client *client) {

	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9v126 *mt9v126 = to_mt9v126(sd);

	printk( KERN_DEBUG "remove\n" );
	v4l2_device_unregister_subdev(sd);

	kfree(mt9v126);
	return 0;
}

static const struct i2c_device_id mt9v126_id[] = { {MT9V126_MODULE_NAME, MT9V126_I2C_ADDR }, { } };
MODULE_DEVICE_TABLE(i2c, mt9v126_id);

static struct i2c_driver mt9v126_i2c_driver = { 
	.driver = { .name = MT9V126_MODULE_NAME, }, 
	.probe = mt9v126_probe, 
	.remove = mt9v126_remove, 
	.id_table = mt9v126_id, 
};

static int __init mt9v126_mod_init(void)
{
	printk( KERN_DEBUG "mod_init\n" );
	return i2c_add_driver(&mt9v126_i2c_driver);
}

static void __exit mt9v126_mod_exit(void)
{
	printk( KERN_DEBUG "mod_exit\n" );
	i2c_del_driver(&mt9v126_i2c_driver);
}

module_init(mt9v126_mod_init);
module_exit(mt9v126_mod_exit);

MODULE_DESCRIPTION("Micron MT9V126 Camera driver");
MODULE_AUTHOR("Klaus Schwarzkopf <schwarzkopf@sensortherm.de>");
MODULE_LICENSE("GPL v2");
