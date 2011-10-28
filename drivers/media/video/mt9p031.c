/*
 * Driver for MT9P031 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2010, RidgeRun <www.ridgerun.com>
 *
 * Author: Miguel Aguilar <miguel.aguilar@ridgerun.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>

#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>

/* mt9p031 i2c address 0x5d
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */

/* mt9p031 selected register addresses */
#define MT9P031_CHIP_VERSION            0x00
#define MT9P031_ROW_START           0x01
#define MT9P031_COLUMN_START            0x02
#define MT9P031_WINDOW_HEIGHT           0x03
#define MT9P031_WINDOW_WIDTH            0x04
#define MT9P031_HORIZONTAL_BLANK        0x05
#define MT9P031_VERTICAL_BLANK          0x06
#define MT9P031_OUTPUT_CONTROL          0x07
#define MT9P031_SHUTTER_WIDTH_UPPER     0x08
#define MT9P031_SHUTTER_WIDTH           0x09
#define MT9P031_PIXEL_CLOCK_CONTROL     0x0A
#define MT9P031_RESTART             0x0B
#define MT9P031_SHUTTER_DELAY           0x0C
#define MT9P031_RESET               0x0D
#define MT9P031_PLL_CONTROL         0x10
#define MT9P031_PLL_CONFIG_1            0x11
#define MT9P031_PLL_CONFIG_2            0x12
#define MT9P031_READ_MODE_1         0x1E
#define MT9P031_READ_MODE_2         0x20
#define MT9P031_ROW_ADDRESS_MODE        0x22
#define MT9P031_COLUMN_ADDRESS_MODE     0x23
#define MT9P031_GREEN_1_GAIN        0x2B
#define MT9P031_BLUE_GAIN           0x2C
#define MT9P031_RED_GAIN            0x2D
#define MT9P031_GREEN_2_GAIN        0x2E
#define MT9P031_GLOBAL_GAIN         0x35
#define MT9P031_TEST_PATTERN_CONTROL        0xA0
#define MT9P031_TEST_PATTERN_GREEN      0xA1
#define MT9P031_TEST_PATTERN_RED        0xA2
#define MT9P031_TEST_PATTERN_BLUE       0xA3
#define MT9P031_TEST_PATTERN_BAR_WIDTH      0xA4
#define MT9P031_CHIP_VERSION_ALT        0xFF

/* Macros for default register values */
#define MT9P031_ROW_SKIP            (0x0036)
#define MT9P031_COLUMN_SKIP         (0x0010)
#define MT9P031_MAX_HEIGHT          (0x0797)
#define MT9P031_MAX_WIDTH           (0x0A1F)
#define MT9P031_MIN_HEIGHT          (0x0002)
#define MT9P031_MIN_WIDTH           (0x0002)
#define MT9P031_DEFAULT_WIDTH           (0x0280) /* 640 */
#define MT9P031_DEFAULT_HEIGHT          (0x01E0) /* 480 */
#define MT9P031_HORIZONTAL_BLANK_DEFAULT    (0x0000)
#define MT9P031_VERTICAL_BLANK_DEFAULT      (0x0019)
#define MT9P031_OUTPUT_CONTROL_DEFAULT      (0x1F82)
#define MT9P031_SHUTTER_WIDTH_UPPER_DEFAULT (0x0000)
#define MT9P031_SHUTTER_WIDTH_DEFAULT       (0x01F4)
#define MT9P031_PIXEL_CLK_CTRL_DEFAULT      (0x0000)
#define MT9P031_FRAME_RESTART_DEFAULT       (0x0000)
#define MT9P031_SHUTTER_DELAY_DEFAULT       (0x0000)
#define MT9P031_PLL_CONTROL_DEFAULT     (0x0050)
#define MT9P031_PLL_CONTROL_POWER_UP        (0x0051)
#define MT9P031_PLL_CONTROL_USE_PLL_CLK     (0x0053)
#define MT9P031_PLL_CONFIG_1_96MHZ      (0x1001)
#define MT9P031_PLL_CONFIG_1_DEFAULT        (0x6404)
#define MT9P031_PLL_CONFIG_2_DEFAULT        (0x0000)
#define MT9P031_PLL_CONFIG_2_96MHZ      (0x0003)
#define MT9P031_READ_MODE1_DEFAULT      (0x4006)
#define MT9P031_READ_MODE2_DEFAULT      (0x0040)
#define MT9P031_ROW_ADDRESS_MODE_DEFAULT    (0x0000)
#define MT9P031_COLUMN_ADDR_MODE_DEFAULT    (0x0000)
#define MT9P031_GREEN_1_GAIN_DEFAULT        (0x0008)
#define MT9P031_BLUE_GAIN_DEFAULT       (0x0008)
#define MT9P031_RED_GAIN_DEFAULT        (0x0008)
#define MT9P031_GREEN_2_GAIN_DEFAULT        (0x0008)
#define MT9P031_GLOBAL_GAIN_DEFAULT     (0x0008)
#define MT9P031_TEST_PATTERN_CONTROL_DEFAULT    (0x0000)
#define MT9P031_TEST_PATTERN_GREEN_DEFAULT  (0x0000)
#define MT9P031_TEST_PATTERN_RED_DEFAULT    (0x0000)
#define MT9P031_TEST_PATTERN_BLUE_DEFAULT   (0x0000)
#define MT9P031_TEST_PATTERN_BAR_WIDTH_DEFAULT  (0x0000)

#define MT9T031_BUS_PARAM   (SOCAM_PCLK_SAMPLE_RISING | \
    SOCAM_PCLK_SAMPLE_FALLING | SOCAM_HSYNC_ACTIVE_HIGH |   \
    SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_HIGH |  \
    SOCAM_MASTER | SOCAM_DATAWIDTH_10)


#define MAX_FRMIVALS            3
#define CEIL(x) ((x - (int)x)==0 ? (int)x : (int)x+1)

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

const struct v4l2_queryctrl *mt9p031_find_qctrl(u32 id);
static int mt9p031_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9p031_set_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9p031_queryctrl(struct v4l2_subdev *, struct v4l2_queryctrl *);

const int hb_min[4][4] = {  {450, 430, 0, 420},
                            {796, 776, 0, 766},
                            {0,   0, 0,   0},
                            {1488,1468,0, 1458} };


static const struct v4l2_fmtdesc mt9p031_formats[] = {
    {
        .index = 0,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .description = "Bayer (sRGB) 10 bit",
        .pixelformat = V4L2_PIX_FMT_SGRBG10,
    },
};
static const unsigned int mt9p031_num_formats = ARRAY_SIZE(mt9p031_formats);

struct capture_size {
    unsigned long width;
    unsigned long height;
};
struct frame_table {
    struct capture_size framesize;
    struct v4l2_fract frameintervals[MAX_FRMIVALS];
    unsigned int num_frmivals;
};

struct exp_data{
    u32 row_time;
    u32 shutter_overlay;
    u32 pix_clock;
};

/* Array of image sizes supported by MT9P031.  These must be ordered from
 * smallest image size to largest.
 */
const static struct frame_table mt9p031_frame_table[] = {
    {
    /* VGA -4X Binning+Skipping */
    .framesize = { 640, 480 },
    .frameintervals = {
        { .numerator =1, .denominator = 30 },
        { .numerator =1, .denominator = 60 },
        },
    .num_frmivals = 2
    },
    {
    /* SVGA-2X Binning+Skipping */
    .framesize = { 800, 600 },
    .frameintervals = {
        { .numerator = 1, .denominator = 65 },
        },
    .num_frmivals = 1
    },
    {
    /* XGA -2X Binning+Skipping */
    .framesize = { 1024, 768 },
    .frameintervals = {
        {  .numerator = 1, .denominator = 47 },
        },
    .num_frmivals = 1
    },
    {
    /* 720P-HDTV -2X Binning+Skipping */
    .framesize = { 1280, 720 },
    .frameintervals = {
        {  .numerator = 1, .denominator = 45 },
        {  .numerator = 1, .denominator = 23 },
        },
    .num_frmivals = 2
    },
    {
    /* SXGA */
    .framesize = { 1280, 1024},
    .frameintervals = {
        {  .numerator = 1, .denominator = 42 },
        },
    .num_frmivals = 1
    },
    {
    /* UXGA */
    .framesize = { 1600, 1200},
    .frameintervals = {
        {  .numerator = 1, .denominator = 31 },
        },
    .num_frmivals = 1
    },
    {
    /* 1080P-HDTV */
    .framesize = { 1920, 1080},
    .frameintervals = {
        {  .numerator = 1, .denominator = 31 },
        },
    .num_frmivals = 1
    },
    {
    /* QXGA */
    .framesize = { 2048, 1536},
    .frameintervals = {
        {  .numerator = 1, .denominator = 21 },
        },
    .num_frmivals = 1
    },
    {
    /* Full Resolution for DM365 that supports maximum 2176x2176  */
    .framesize = { 2176, 1944},
    .frameintervals = {
        {  .numerator = 1, .denominator = 14 },
        },
    .num_frmivals = 1
    },
    {
    /* Full Resolution */
    .framesize = { 2592, 1944},
    .frameintervals = {
        {  .numerator = 1, .denominator = 14 },
        },
    .num_frmivals = 1
    }
};
static const unsigned int mt9p031_num_frmsizes = ARRAY_SIZE(mt9p031_frame_table);

static const struct v4l2_queryctrl mt9p031_controls[] = {
    {
        .id     = V4L2_CID_VFLIP,
        .type       = V4L2_CTRL_TYPE_BOOLEAN,
        .name       = "Flip Vertically",
        .minimum    = 0,
        .maximum    = 1,
        .step       = 1,
        .default_value  = 0,
    }, {
        .id     = V4L2_CID_HFLIP,
        .type       = V4L2_CTRL_TYPE_BOOLEAN,
        .name       = "Flip Horizontally",
        .minimum    = 0,
        .maximum    = 1,
        .step       = 1,
        .default_value  = 0,
    }, {
        .id     = V4L2_CID_GAIN,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Gain",
        .minimum    = 0,
        .maximum    = 1024,
        .step       = 1,
        .default_value  = 8,
        .flags      = V4L2_CTRL_FLAG_SLIDER,
    }, {
        .id     = V4L2_CID_EXPOSURE,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Exposure",
        .minimum    = 1,
        .maximum    = (0x7fffffff),/*Original 255*/
        .step       = 1,
        .default_value  = 255,
        .flags      = V4L2_CTRL_FLAG_SLIDER,
    }, {
        .id     = V4L2_CID_EXPOSURE_AUTO,
        .type       = V4L2_CTRL_TYPE_BOOLEAN,
        .name       = "Automatic Exposure",
        .minimum    = 0,
        .maximum    = 1,
        .step       = 1,
        .default_value  = 0,
    }, {
        .id     = V4L2_CID_RED_BALANCE,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Red Balance",
        .minimum    = 1,
        .maximum    = 1024,
        .step       = 1,
        .default_value  = 8
    }, {
        .id     = V4L2_CID_BRIGHTNESS,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Brightness (Green 1 Balance)",
        .minimum    = 1,
        .maximum    = 1024,
        .step       = 1,
        .default_value  = 8
    }, {
        .id     = V4L2_CID_AUTOGAIN,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Auto Gain (Green 2 Balance)",
        .minimum    = 1,
        .maximum    = 1024,
        .step       = 1,
        .default_value  = 8
    }, {
        .id     = V4L2_CID_BLUE_BALANCE,
        .type       = V4L2_CTRL_TYPE_INTEGER,
        .name       = "Blue Balance",
        .minimum    = 1,
        .maximum    = 1024,
        .step       = 1,
        .default_value  = 8
    },
};

static const unsigned int mt9p031_num_controls = ARRAY_SIZE(mt9p031_controls);

struct mt9p031 {
    struct v4l2_subdev sd;
    int model;  /* V4L2_IDENT_MT9T031* codes from v4l2-chip-ident.h */
    unsigned char autoexposure;
    u16 xskip;
    u16 yskip;
    u32 xbin;
    u32 ybin;
    u32 width;
    u32 height;
    unsigned short x_min;           /* Camera capabilities */
    unsigned short y_min;
    unsigned short x_current;       /* Current window location */
    unsigned short y_current;
    unsigned short width_min;
    unsigned short width_max;
    unsigned short height_min;
    unsigned short height_max;
    unsigned short y_skip_top;      /* Lines to skip at the top */
    unsigned short gain;
    u32 exposure;
    unsigned short mirror_column;
    unsigned short mirror_row;
    struct exp_data exp;
};

static inline struct mt9p031 *to_mt9p031(struct v4l2_subdev *sd)
{
    return container_of(sd, struct mt9p031, sd);
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
    s32 data;

    data = i2c_smbus_read_word_data(client, reg);
    return data < 0 ? data : swab16(data);
}

static int reg_write(struct i2c_client *client, const u8 reg,
             const u16 data)
{
    //int ret;

    //ret = reg_read(client, reg);
    //printk("\n***Register:0x%x actualvalue:0x%x, Value to be write:0x%x",reg,ret,data);
    return i2c_smbus_write_word_data(client, reg, swab16(data));
}

static int reg_set(struct i2c_client *client, const u8 reg,
           const u16 data)
{
    int ret;

    ret = reg_read(client, reg);
    if (ret < 0)
        return ret;
    return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg,
             const u16 data)
{
    int ret;

    ret = reg_read(client, reg);
    if (ret < 0)
        return ret;
    return reg_write(client, reg, ret & ~data);
}

static int set_shutter(struct v4l2_subdev *sd, const u32 data)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret;
    ret = reg_write(client, MT9P031_SHUTTER_WIDTH_UPPER, data >> 16);

    if (ret >= 0)
        ret = reg_write(client, MT9P031_SHUTTER_WIDTH, data & 0xffff);

    return ret;
}

static int get_shutter(struct v4l2_subdev *sd, u32 *data)
{
    int ret;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    ret = reg_read(client, MT9P031_SHUTTER_WIDTH_UPPER);
    *data = ret << 16;

    if (ret >= 0)
        ret = reg_read(client, MT9P031_SHUTTER_WIDTH);
    *data |= ret & 0xffff;
    return ret < 0 ? ret : 0;
}

void calc_shutter(struct v4l2_subdev *sd,
    struct mt9p031 *mt9p031)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    u32 pix_clk, shutter_delay, so, max, hb, hbmin, hb_max, w, row_t, column_size;
    /* Pixclk Period = 10.42ns, fixed point 24:8 in ns */
    pix_clk = 2667;
    /* Shutter Delay, increasing this decrease the exposure time*/
    shutter_delay = reg_read(client, MT9P031_SHUTTER_DELAY) + 1;
    /* Shutter overhead */
    so = 208 * (mt9p031->xbin) + 98 + min((int)shutter_delay, 1504) - 94;
    /* Output image width */
    column_size = reg_read(client, MT9P031_WINDOW_WIDTH);
    w = 2 * CEIL((column_size + 1)/(2 * (mt9p031->xskip)));
    /* Horizontal blanking */
    hb = reg_read(client, MT9P031_HORIZONTAL_BLANK) + 1;
    /* Minimun horizontal blanking*/
    hbmin = hb_min[mt9p031->xbin-1][mt9p031->ybin-1];
    /* Row time calculation, fixed point 24:8 in us */
    hb_max = max(hb, hbmin);
    max = max((w/2 + hb_max),(41 + 346*(mt9p031->xbin) + 99));
    row_t = (2 * pix_clk * max)/1000;

    mt9p031->exp.row_time = row_t;
    mt9p031->exp.shutter_overlay = so;
    mt9p031->exp.pix_clock = pix_clk;
    return;
}

/* See Datasheet Table 17, Gain settings.
 * Since posible values goes from 1-128, and the received values goes from 8-1024
 * map  [1-1024] to [1-128] => Gain = ctrl->value/8
 * if Gain -> [1-8]  =>  AnalogGain -> [8-32] ,  steps of 0.125
 * if Gain -> [4.25-8] => AnalogGain -> [17-32] & AnalogMultipier=1, steps of 0.25
 * If Gain -> [9-128] => AnalogGain = 32 & AnalogMultipier=1 & DigitalGain=>[1-120], steps of 1
 */
int calc_gain(u32 data)
{
    int gain;
    if (data <= 32)
        /* received gain 9..32 -> 9..32 */
        gain = data;
    else if (data <= 64)
        /* received gain 33..64 -> 0x51..0x60 */
        gain = ((data + 1) >> 1) | 0x40;
    else if(data <= 1024)
        /* received gain 65..1024 -> (1..120) << 8 +0x60*/
        gain =(((data - 64 + 4) * 32) & 0xff00) | 0x60;
    else
        gain = 0x7860;

    return gain;
}
/*
 * ======== mt9p031_setpll =========
*/
/*   Function to set the clock pll   Freq_ext = 24Mhz */
/*   Freq = Freq_ext * m_factor / (n_factor + 1) / (p1_factor + 1) */
/*   2MHz < Freq_ext / (n_factor+1) < 13.5MHz */
/*   180 MHz < (Freq_ext * m_factor) / (n_factor+1)< 360 MHz  */

static int mt9p031_setpll(struct i2c_client *client, unsigned char m_factor,
                unsigned char n_factor, unsigned char p1_factor)
{
    int ret;

    ret = reg_write(client, MT9P031_PLL_CONTROL,
                  MT9P031_PLL_CONTROL_POWER_UP);

    if (ret >= 0)
        ret = reg_write(client, MT9P031_PLL_CONFIG_1,
                  (m_factor << 8) | (n_factor & 0x003f));
    if (ret >= 0)
        ret = reg_write(client, MT9P031_PLL_CONFIG_2,
                  p1_factor & 0x001f);

    msleep(10);

    if (ret >= 0)
        ret = reg_write(client, MT9P031_PLL_CONTROL,
                  MT9P031_PLL_CONTROL_USE_PLL_CLK);

    return ret >= 0 ? 0 : -EIO;
}

static int mt9p031_init(struct v4l2_subdev *sd, u32 val)
{
    int ret, shutter;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct v4l2_control ctrl;
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    u32 i;
    const u16 vblank = MT9P031_VERTICAL_BLANK_DEFAULT;

    /* Disable chip output, synchronous option update */
    ret = reg_write(client, MT9P031_RESET, 1);
    if (ret >= 0)
        ret = reg_write(client, MT9P031_RESET, 0);

    /*Soft Standby*/
    if (ret >= 0)
        ret = reg_clear(client, MT9P031_OUTPUT_CONTROL, 2);

    /*if (ret >= 0)
        ret = reg_set(client, MT9P031_OUTPUT_CONTROL, 1);*/

    /* Set defaults of the controls*/
    for (i = 0; i < mt9p031_num_controls; i++) {
        if (mt9p031_controls[i].id == V4L2_CID_EXPOSURE)
            i++;
        ctrl.id = mt9p031_controls[i].id;
        ctrl.value = mt9p031_controls[i].default_value;
        mt9p031_set_control(sd,&ctrl);
    }
    /*if (ret >= 0)
        ret = reg_clear(client, MT9P031_OUTPUT_CONTROL, 1);*/
    shutter = 479 + vblank;
    ret = set_shutter(sd, shutter);
    calc_shutter(sd, mt9p031);
    mt9p031->exposure = shutter * mt9p031->exp.row_time -
       (mt9p031->exp.shutter_overlay * 2 * mt9p031->exp.pix_clock)/1000;
    return ret >= 0 ? 0 : -EIO;
}

static int mt9p031_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret;

    /* Switch to master "normal" mode - StreamOn*/
    if (enable) {

        /*Enable Chip*/
        if (reg_set(client, MT9P031_OUTPUT_CONTROL, 2) < 0)
            return -EIO;

        /*Restart Pause*/
        ret = reg_set(client, MT9P031_RESTART,2);
        ret = reg_set(client, MT9P031_RESTART,1);
        /*Setup PLL to generate  96MHz from extclk at 24MHz*/
        ret = mt9p031_setpll(client, 16, 1, 1);
        /*Resume*/
        ret = reg_clear(client, MT9P031_RESTART,2);

    } else {
    /* Switch to master "" mode - StreamOff*/
        /*Restart Pause*/
        ret = reg_set(client, MT9P031_RESTART,2);
        ret = reg_set(client, MT9P031_RESTART,1);

        if (reg_clear(client, MT9P031_OUTPUT_CONTROL, 2) < 0)
            return -EIO;

        /*Resume*/
        ret = reg_clear(client, MT9P031_RESTART,2);
    }
    return 0;
}

/* Round up minima and round down maxima */
static void recalculate_limits(struct mt9p031 *mt9p031,
                   u16 xskip, u16 yskip)
{

    mt9p031->x_min = (MT9P031_COLUMN_SKIP + 2 * xskip - 1) & ~(2 * xskip - 1);
    mt9p031->y_min = (MT9P031_ROW_SKIP + 2 * yskip - 1) & ~(2 * yskip - 1);
    mt9p031->width_min = (MT9P031_MIN_WIDTH + 2 * xskip - 1) & ~(2 * xskip - 1);
    mt9p031->height_min = (MT9P031_MIN_HEIGHT + 2 * yskip - 1) & ~(2 * yskip - 1);
    mt9p031->width_max = MT9P031_MAX_WIDTH / xskip;
    mt9p031->height_max = MT9P031_MAX_HEIGHT / yskip;
}

const struct v4l2_queryctrl *mt9p031_find_qctrl(u32 id)
{
    int i;

    for (i = 0; i < mt9p031_num_controls; i++) {
        if (mt9p031_controls[i].id == id){
            return &mt9p031_controls[i];
        }
    }
    return NULL;
}

static int mt9p031_set_params(struct v4l2_subdev *sd,
                  struct v4l2_rect *rect, u16 xskip, u16 yskip)
{
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret, shutter = 1;
    u16 xbin=1, ybin=1, width, height, left, top;
    const u16 hblank = MT9P031_HORIZONTAL_BLANK_DEFAULT;
    const u16 vblank = MT9P031_VERTICAL_BLANK_DEFAULT;

    /* Make sure we don't exceed sensor limits */
    if (rect->left + rect->width > mt9p031->width_max)
        rect->left =
        (mt9p031->width_max - rect->width) / 2 + mt9p031->x_min;

    if (rect->top + rect->height > mt9p031->height_max)
        rect->top =
        (mt9p031->height_max - rect->height) / 2 + mt9p031->y_min;

    width = rect->width * xskip;
    height = rect->height * yskip;
    left = rect->left;
    top = rect->top;
    /* See Datasheet Table 14, Legal Skip/Bin Values
     * Remember for xskip=2X,Row Skip=1,
     *          for xskip=3X,Row Skip=2, and so on.
     */
    if(xskip & 1)
        xbin = 1;
    else if (xskip & 2)
        xbin = 2;
    else if (xskip & 4)
        xbin = 4;

    if(yskip & 1)
        ybin = 1;
    else if (yskip & 2)
        ybin = 2;
    else if (yskip & 4)
        ybin = 4;

    v4l2_dbg(1, debug, sd, "xskip %u, width %u/%u, yskip %u,"
        "height %u/%u\n", xskip, width, rect->width, yskip,
        height, rect->height);

    /*Rounded down to the nearest multiple of 4 times the bin factor*/
    left = ((left) & ~(2 * xbin -1)) + 1 * xbin * mt9p031->mirror_row;
    top = ((top) & ~(2 * ybin -1)) + 1 * ybin * mt9p031->mirror_column;

    /* Disable register update, reconfigure atomically */
    ret = reg_set(client, MT9P031_OUTPUT_CONTROL, 1);
    if (ret < 0)
        return ret;

    /* Blanking and start values - default... */
    ret = reg_write(client, MT9P031_HORIZONTAL_BLANK, hblank);
    if (ret >= 0)
        ret = reg_write(client, MT9P031_VERTICAL_BLANK, vblank);

    if (yskip != mt9p031->yskip || xskip != mt9p031->xskip) {
        /* Binning, skipping */
        if (ret >= 0)
            ret = reg_write(client, MT9P031_COLUMN_ADDRESS_MODE,
                    ((xbin - 1) << 4) | (xskip - 1));
        if (ret >= 0)
            ret = reg_write(client, MT9P031_ROW_ADDRESS_MODE,
                    ((ybin - 1) << 4) | (yskip - 1));
    }

    v4l2_dbg(1, debug, sd, "new physical left %u, top %u\n", left, top);

    /* The caller provides a supported format, as guaranteed by
     * icd->try_fmt_cap(), soc_camera_s_crop() and soc_camera_cropcap() */
    if (ret >= 0)
        ret = reg_write(client, MT9P031_COLUMN_START, left);
    if (ret >= 0)
        ret = reg_write(client, MT9P031_ROW_START, top);
    if (ret >= 0)
        ret = reg_write(client, MT9P031_WINDOW_WIDTH, width - 1);
    if (ret >= 0)
        ret = reg_write(client, MT9P031_WINDOW_HEIGHT,
                height + mt9p031->y_skip_top - 1);

    if (ret >= 0 && mt9p031->autoexposure) {
        shutter = rect->height-1 + vblank;
        ret = set_shutter(sd, shutter);
    }
    if (ret >= 0)
        ret = reg_set(client, MT9P031_READ_MODE_2, 0x0020);

    /* Re-enable register update, commit all changes */
    if (ret >= 0) {
        ret = reg_clear(client, MT9P031_OUTPUT_CONTROL, 1);
        /* update the values */
        mt9p031->width  = rect->width,
        mt9p031->height = rect->height,
        mt9p031->x_current = rect->left;
        mt9p031->y_current = rect->top;
        mt9p031->ybin = ybin;
        mt9p031->xbin = xbin;
        mt9p031->xskip = xskip;
        mt9p031->yskip = yskip;

        if (mt9p031->autoexposure) {
            calc_shutter(sd, mt9p031);
            mt9p031->exposure = shutter * mt9p031->exp.row_time -
            (mt9p031->exp.shutter_overlay * 2 * mt9p031->exp.pix_clock)/1000;
        }
    }
    return ret < 0 ? ret : 0;

}

static int mt9p031_set_fmt(struct v4l2_subdev *sd,
               struct v4l2_format *f)
{
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    int ret;
    u16 xskip, yskip;
    struct v4l2_rect rect = {
        .left   = mt9p031->x_current,
        .top    = mt9p031->y_current,
        .width  = f->fmt.pix.width,
        .height = f->fmt.pix.height,
    };

    /*
     * try_fmt has put rectangle within limits.
     * S_FMT - use binning and skipping for scaling, recalculate
     * limits, used for cropping
     */
    /* Is this more optimal than just a division */
    for (xskip = 8; xskip > 1; xskip--)
        if (rect.width * xskip <= MT9P031_MAX_WIDTH)
            break;

    for (yskip = 8; yskip > 1; yskip--)
        if (rect.height * yskip <= MT9P031_MAX_HEIGHT)
            break;

    recalculate_limits(mt9p031, xskip, yskip);
    ret = mt9p031_set_params(sd, &rect, xskip, yskip);
    return ret;
}

static int mt9p031_try_fmt(struct v4l2_subdev *sd,
               struct v4l2_format *f)
{
    struct v4l2_pix_format *pix = &f->fmt.pix;

    if (pix->height < MT9P031_MIN_HEIGHT)
        pix->height = MT9P031_MIN_HEIGHT;
    if (pix->height > MT9P031_MAX_HEIGHT)
        pix->height = MT9P031_MAX_HEIGHT;
    if (pix->width < MT9P031_MIN_WIDTH)
        pix->width = MT9P031_MIN_WIDTH;
    if (pix->width > MT9P031_MAX_WIDTH)
        pix->width = MT9P031_MAX_WIDTH;

    pix->width &= ~0x01; /* has to be even */
    pix->height &= ~0x01; /* has to be even */
    return 0;
}

static int mt9p031_enum_framesizes(struct v4l2_subdev *sd,
                struct v4l2_frmsizeenum *frms)
{
    int ifmt;
    for (ifmt = 0; ifmt < mt9p031_num_formats; ifmt++) {
        if (frms->pixel_format == mt9p031_formats[ifmt].pixelformat) {
            break;
        }
    }
    /* Is requested pixelformat not found on sensor? */
    if (ifmt == mt9p031_num_formats) {
        v4l2_err(sd, "pixel format %d, not found on sensor\n",frms->pixel_format);
        return -EINVAL;
    }

    /* Do we already reached all discrete framesizes? */
    if (frms->index >= mt9p031_num_frmsizes){
        return -EINVAL;
    }

    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = mt9p031_frame_table[frms->index].framesize.width;
    frms->discrete.height = mt9p031_frame_table[frms->index].framesize.height;

    return 0;
}

static int mt9p031_enum_frameintervals(struct v4l2_subdev *sd,
                struct v4l2_frmivalenum *frmi)
{
    int ifmt;
    struct v4l2_frmsizeenum frms;

    for (ifmt = 0; ifmt < mt9p031_num_formats; ifmt++) {
        if (frmi->pixel_format == mt9p031_formats[ifmt].pixelformat) {
            break;
        }
    }
    /* Is requested pixelformat not found on sensor? */
    if (ifmt == mt9p031_num_formats) {
        v4l2_err(sd, "pixel format %d, not found on sensor\n",frms.pixel_format);
        return -EINVAL;
    }

    frms.index = 0;
    frms.pixel_format = frmi->pixel_format;

    /* Do we already reached all discrete framesizes? */
    while (mt9p031_enum_framesizes(sd,&frms) >= 0) {
        if (((frmi->width == frms.discrete.width) &&
            (frmi->height == frms.discrete.height))){
            break;
        }
        frms.index++;
    }
    if (frms.index >= mt9p031_num_frmsizes){
        v4l2_err(sd, "Frame size:width=%d and height=%d, not supported on sensor\n",
                frmi->width,frmi->height);
        return -EINVAL;
    }

    if (frmi->index >= mt9p031_frame_table[frms.index].num_frmivals)
        return -EINVAL;

    frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frmi->discrete.numerator =
                mt9p031_frame_table[frms.index].frameintervals[frmi->index].numerator;
    frmi->discrete.denominator =
                mt9p031_frame_table[frms.index].frameintervals[frmi->index].denominator;

    return 0;
}

static int mt9p031_get_chip_id(struct v4l2_subdev *sd,
                   struct v4l2_dbg_chip_ident *id)
{
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);;

    if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
        return -EINVAL;

    if (id->match.addr != client->addr)
        return -ENODEV;

    id->ident   = mt9p031->model;
    id->revision    = 0;

    return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9p031_get_register(struct v4l2_subdev *sd,
                struct v4l2_dbg_register *reg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);;
    struct mt9p031 *mt9p031 = to_mt9p031(sd);

    if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
        return -EINVAL;

    if (reg->match.addr != client->addr)
        return -ENODEV;

    reg->val = reg_read(client, reg->reg);

    if (reg->val > 0xffff)
        return -EIO;

    return 0;
}

static int mt9p031_set_register(struct v4l2_subdev *sd,
                struct v4l2_dbg_register *reg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct mt9p031 *mt9p031 = to_mt9p031(sd);

    if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
        return -EINVAL;

    if (reg->match.addr != client->addr)
        return -ENODEV;

    if (reg_write(client, reg->reg, reg->val) < 0)
        return -EIO;

    return 0;
}
#endif


static const struct v4l2_subdev_core_ops mt9p031_core_ops = {
    .g_chip_ident = mt9p031_get_chip_id,
    .init = mt9p031_init,
    .queryctrl = mt9p031_queryctrl,
    .g_ctrl = mt9p031_get_control,
    .s_ctrl = mt9p031_set_control,
#ifdef CONFIG_VIDEO_ADV_DEBUG
    .get_register = mt9p031_get_register,
    .set_register = mt9p031_set_register,
#endif
};

static const struct v4l2_subdev_video_ops mt9p031_video_ops = {
    .s_fmt = mt9p031_set_fmt,
    .try_fmt = mt9p031_try_fmt,
    .s_stream = mt9p031_s_stream,
    .enum_framesizes = mt9p031_enum_framesizes,
    .enum_frameintervals = mt9p031_enum_frameintervals,
};

static const struct v4l2_subdev_ops mt9p031_ops = {
    .core = &mt9p031_core_ops,
    .video = &mt9p031_video_ops,
};

static int mt9p031_queryctrl(struct v4l2_subdev *sd,
                struct v4l2_queryctrl *qctrl)
{
    const struct v4l2_queryctrl *temp_qctrl;

    temp_qctrl = mt9p031_find_qctrl(qctrl->id);
    if (!temp_qctrl) {
        v4l2_dbg(1, debug,sd, "control id %d not supported", qctrl->id);
        return -EINVAL;
    }
    memcpy(qctrl, temp_qctrl, sizeof(*qctrl));
    return 0;
}

static int mt9p031_get_control(struct v4l2_subdev *sd,
                   struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    int data;
    u8 reg = -1;
    bool gain = false;

    switch (ctrl->id) {
    case V4L2_CID_VFLIP:
        data = reg_read(client, MT9P031_READ_MODE_2);
        if (data < 0)
            return -EIO;
        ctrl->value = !!(data & 0x8000);
        break;
    case V4L2_CID_HFLIP:
        data = reg_read(client, MT9P031_READ_MODE_2);
        if (data < 0)
            return -EIO;
        ctrl->value = !!(data & 0x4000);
        break;
    case V4L2_CID_EXPOSURE:
        ctrl->value = mt9p031->exposure;
        break;
    case V4L2_CID_EXPOSURE_AUTO:
        ctrl->value = mt9p031->autoexposure;
        break;
    case V4L2_CID_RED_BALANCE:
        reg = MT9P031_RED_GAIN;
        gain = true;
        break;
    case V4L2_CID_BLUE_BALANCE:
        reg = MT9P031_BLUE_GAIN;
        gain = true;
        break;
    case V4L2_CID_BRIGHTNESS:
        reg = MT9P031_GREEN_1_GAIN;
        gain = true;
        break;
    case V4L2_CID_AUTOGAIN:
        reg = MT9P031_GREEN_2_GAIN;
        gain = true;
        break;
    }
    if (gain){
        data = reg_read(client, reg);
        if ( (data & 0x7f40)  == 0 )
            ctrl->value = data;
        else if ((data & 0x7f00) == 0)
            ctrl->value = ((data & 0x003f) << 1);
        else
            ctrl->value = ((data & 0xff00)>>5) + 64;
    }
    return 0;
}

static int mt9p031_set_control(struct v4l2_subdev *sd,
                   struct v4l2_control *ctrl)
{
    struct mt9p031 *mt9p031 = to_mt9p031(sd);
    const struct v4l2_queryctrl *qctrl = NULL;
    int data = 0, ret = 0;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    u32 old, sw;

    if (NULL == ctrl)
        return -EINVAL;

    ret = reg_set(client, MT9P031_OUTPUT_CONTROL, 1);

    qctrl = mt9p031_find_qctrl(ctrl->id);
    if (!qctrl) {
        v4l2_err(sd, "control id %d not supported", ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id) {
    case V4L2_CID_VFLIP:
        if (ctrl->value){
            data = reg_set(client, MT9P031_READ_MODE_2, 0x8000);
            mt9p031->mirror_column = 1;
        }else{
            data = reg_clear(client, MT9P031_READ_MODE_2, 0x8000);
            mt9p031->mirror_column = 0;
        }
        if (data < 0)
            return -EIO;
        break;
    case V4L2_CID_HFLIP:
        if (ctrl->value){
            data = reg_set(client, MT9P031_READ_MODE_2, 0x4000);
            mt9p031->mirror_row = 1;
        }else{
            data = reg_clear(client, MT9P031_READ_MODE_2, 0x4000);
            mt9p031->mirror_row = 0;
        }
        if (data < 0)
            return -EIO;
        break;
    case V4L2_CID_GAIN:
        /* This control will be used to modify Gain. */
        if (ctrl->value > qctrl->maximum || ctrl->value < qctrl->minimum){
            printk("I receive a value that exceeds the range:%d\n",(int)ctrl->value);
            return -EINVAL;
        }
        data = calc_gain(ctrl->value);
        v4l2_dbg(1, debug, sd, "Setting gain from 0x%x to 0x%x\n",
                 reg_read(client, MT9P031_GLOBAL_GAIN), data);
        ret= reg_write(client, MT9P031_GLOBAL_GAIN, data);
        if (ret < 0)
            return -EIO;
        /* Success */
        mt9p031->gain = ctrl->value;
        break;
    case V4L2_CID_EXPOSURE:
        /* mt9p031 has maximum == default */
        if (ctrl->value > qctrl->maximum ||
            ctrl->value < qctrl->minimum){
            printk("Exposure range is exceed:%d\n",(int)ctrl->value);
            return -EINVAL;
        }else {
            get_shutter(sd, &old);
            mt9p031->exposure = ctrl->value;
            /* Shutter width calculation,
             * ctrl->value must be in a fixed point 24:8 in us*/
            sw = (ctrl->value + (mt9p031->exp.shutter_overlay*2*
                mt9p031->exp.pix_clock)/1000)/mt9p031->exp.row_time;
            /* Shutter width must be greater than 1*/
            if (sw < 1)
                sw = 1;
            ret = set_shutter(sd, sw);
            if (ret < 0){
                printk("I could not write shutter width register\n");
                return -EIO;
            }
            v4l2_dbg(1, debug, sd,
                "Setting shutter width from %u to %u\n",
                old, ctrl->value);
            mt9p031->autoexposure = 0;
        }
        break;
    case V4L2_CID_EXPOSURE_AUTO:
        if (ctrl->value) {
            const u16 vblank = MT9P031_VERTICAL_BLANK_DEFAULT;
            const u32 shutter_max = MT9P031_MAX_HEIGHT + vblank;
            if (set_shutter(sd, mt9p031->height +
                    mt9p031->y_skip_top + vblank) < 0)
                return -EIO;

            qctrl = mt9p031_find_qctrl(V4L2_CID_EXPOSURE);
            mt9p031->exposure
             =
                (shutter_max / 2 + (mt9p031->height +
                mt9p031->y_skip_top + vblank - 1) *
                (qctrl->maximum - qctrl->minimum)) /
                shutter_max + qctrl->minimum;
            mt9p031->autoexposure = 1;
        } else{
            mt9p031->autoexposure = 0;
        }break;
    case V4L2_CID_RED_BALANCE:
    case V4L2_CID_BLUE_BALANCE:
    case V4L2_CID_BRIGHTNESS:
    case V4L2_CID_AUTOGAIN:
        /* This control will be used to modify Gain */
        if (ctrl->value > qctrl->maximum || ctrl->value < qctrl->minimum){
            printk("I receive a value that exceeds the range:%d\n",(int)ctrl->value);
            return -EINVAL;
        }
        data = calc_gain(ctrl->value);
        v4l2_dbg(1, debug, sd, "Setting gain %d\n", data);
        switch (ctrl->id) {
            case V4L2_CID_RED_BALANCE:
                ret = reg_write(client, MT9P031_RED_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Red Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_BLUE_BALANCE:
                ret = reg_write(client, MT9P031_BLUE_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Blue Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_BRIGHTNESS:
                ret = reg_write(client, MT9P031_GREEN_1_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Green1 Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_AUTOGAIN:
                ret = reg_write(client, MT9P031_GREEN_2_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Green2 Gain register\n");
                    return -EIO;
                }
                break;
        }
        break;
    }
    reg_clear(client, MT9P031_OUTPUT_CONTROL, 1);

    return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int mt9p031_detect(struct i2c_client *client, int *model)
{
    s32 data;

    /* Enable the chip */
//  data = reg_write(client, MT9P031_CHIP_ENABLE, 1);
//  dev_dbg(&client->dev, "write: %d\n", data);

    /* Read out the chip version register */
    data = reg_read(client, MT9P031_CHIP_VERSION);

    switch (data) {
    case 0x1801:
        //*model = V4L2_IDENT_MT9P031;
        break;
    default:
        dev_err(&client->dev,
            "No MT9P031 chip detected, register read %x\n", data);
        return -ENODEV;
    }

    dev_info(&client->dev, "Detected a MT9P031 chip ID %x\n", data);
    return 0;
}

static int mt9p031_probe(struct i2c_client *client,
             const struct i2c_device_id *did)
{
    struct mt9p031 *mt9p031;
    struct v4l2_subdev *sd;
    int pclk_pol;
    int ret;
    if (!i2c_check_functionality(client->adapter,
                     I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_warn(&client->dev,
             "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
        return -EIO;
    }

    if (!client->dev.platform_data) {
        dev_err(&client->dev, "No platform data!!\n");
        return -ENODEV;
    }

    pclk_pol = (int)client->dev.platform_data;

    mt9p031 = kzalloc(sizeof(struct mt9p031), GFP_KERNEL);
    if (!mt9p031)
        return -ENOMEM;

    ret = mt9p031_detect(client, &mt9p031->model);
    if (ret)
        goto clean;

    mt9p031->x_min      = MT9P031_COLUMN_SKIP;
    mt9p031->y_min      = MT9P031_ROW_SKIP;
    mt9p031->width      = MT9P031_DEFAULT_WIDTH;
    mt9p031->height     = MT9P031_DEFAULT_HEIGHT;
    mt9p031->x_current  = mt9p031->x_min;
    mt9p031->y_current  = mt9p031->y_min;
    mt9p031->width_min  = MT9P031_MIN_WIDTH;
    mt9p031->width_max  = MT9P031_MAX_WIDTH;
    mt9p031->height_min = MT9P031_MIN_HEIGHT;
    mt9p031->height_max = MT9P031_MAX_HEIGHT;
    mt9p031->y_skip_top = 10; //Originally it had 10, minimun value(6)which works
    mt9p031->autoexposure = 1;
    mt9p031->xskip = 1;
    mt9p031->yskip = 1;
    mt9p031->xbin = 1;
    mt9p031->ybin = 1;
    mt9p031->mirror_column = 0;
    mt9p031->mirror_row = 0;

    /* Register with V4L2 layer as slave device */
    sd = &mt9p031->sd;
    v4l2_i2c_subdev_init(sd, client, &mt9p031_ops);
    if (!pclk_pol)
        reg_clear(v4l2_get_subdevdata(sd),
              MT9P031_PIXEL_CLOCK_CONTROL, 0x8000);
    else
        reg_set(v4l2_get_subdevdata(sd),
        MT9P031_PIXEL_CLOCK_CONTROL, 0x8000);

    ret = mt9p031_init(sd,1);
    v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);
    return 0;

clean:
    kfree(mt9p031);
    return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct mt9p031 *mt9p031 = to_mt9p031(sd);

    v4l2_device_unregister_subdev(sd);

    kfree(mt9p031);
    return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
    { "mt9p031", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
    .driver = {
        .name = "mt9p031",
    },
    .probe      = mt9p031_probe,
    .remove     = mt9p031_remove,
    .id_table   = mt9p031_id,
};

static int __init mt9p031_mod_init(void)
{
    return i2c_add_driver(&mt9p031_i2c_driver);
}
module_init(mt9p031_mod_init);

static void __exit mt9p031_mod_exit(void)
{
    i2c_del_driver(&mt9p031_i2c_driver);
}
module_exit(mt9p031_mod_exit);

MODULE_DESCRIPTION("Micron MT9P031 Camera driver");
MODULE_AUTHOR("Miguel Aguilar <miguel.aguilar@ridgerun.com>");
MODULE_LICENSE("GPL");
