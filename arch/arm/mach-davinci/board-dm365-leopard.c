/*
 * DM365 Leopard Board
 *
 * Derived from: arch/arm/mach-davinci/board-dm365-evm.c
 * RidgeRun Copyright (C) 2010.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 * Included Files
 **************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/cputype.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>
#include <media/mt9v126.h>

#define VPSS_CLK_CTRL						0x01C40044
#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

static struct snd_platform_data dm365_leopard_snd_data = {
	.asp_chan_q = EVENTQ_3,
};

static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	/*
	{
		I2C_BOARD_INFO("ths7303", 0x2c),
	}
	*/
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
};
/* Input available at the mt9p031 */
static struct v4l2_input camera_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
	}
};


static struct vpfe_subdev_info vpfe_sub_devs[] = {
#ifdef CONFIG_SOC_CAMERA_MT9P031
	{
		.module_name = "mt9p031",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9P031,
		.num_inputs = ARRAY_SIZE(camera_inputs),
		.inputs = camera_inputs,
		.ccdc_if_params = {
			.if_type = V4L2_MBUS_FMT_SBGGR10_1X10,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("mt9p031", 0x48),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	}
#endif
#if defined(CONFIG_SOC_CAMERA_MT9V126) || defined(CONFIG_SOC_CAMERA_MT9V126_MODULE)
	{
		.module_name = MT9V126_MODULE_NAME,
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9V126,
		.num_inputs = ARRAY_SIZE(camera_inputs),
		.inputs = camera_inputs,
		.ccdc_if_params = {
			.if_type = V4L2_MBUS_FMT_YUYV8_2X8,//VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO(MT9V126_MODULE_NAME, MT9V126_I2C_ADDR),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	},
#endif
};

/* Set the input  for TVPxxx/MTxxxx sensors */
static int dm365leopard_setup_video_input(enum vpfe_subdev_id id)
{
	const char *label;

	u32 vpss_clk_ctrl=0;
	switch (id) {

		case VPFE_SUBDEV_MT9P031:
			label = "HD imager-MT9P031";
			break;
		case VPFE_SUBDEV_MT9V126:
			label = "VGA imager-MT9V126";
			//VPSS_CLK_CTRL.PCLK_INV = 1 for BT.656
			vpss_clk_ctrl =  __raw_readl(IO_ADDRESS(VPSS_CLK_CTRL));
			__raw_writel(vpss_clk_ctrl | (1<<2), IO_ADDRESS(VPSS_CLK_CTRL));

			break;
		default:
			return 0;
	}

	pr_info("Leopard: switch to %s video input\n", label);
	return 0;
}

static struct vpfe_config vpfe_cfg = {
       .setup_input = dm365leopard_setup_video_input,
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 Leopard",
/*       .ccdc = "DM365 ISIF", */
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

#define VENC_STD_ALL	(V4L2_STD_NTSC | V4L2_STD_PAL)

/* venc standards timings */
static struct vpbe_enc_mode_info vbpe_enc_std_timings[] = {
	{
		.name		= "ntsc",
		.timings_type	= VPBE_ENC_STD,
		.timings	= {V4L2_STD_525_60},
		.interlaced	= 1,
		.xres		= 720,
		.yres		= 480,
		.aspect		= {11, 10},
		.fps		= {30000, 1001},
		.left_margin	= 0x79,
		.right_margin	= 0,
		.upper_margin	= 0x10,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.flags		= 0,
	},
	{
		.name		= "pal",
		.timings_type	= VPBE_ENC_STD,
		.timings	= {V4L2_STD_625_50},
		.interlaced	= 1,
		.xres		= 720,
		.yres		= 576,
		.aspect		= {54, 59},
		.fps		= {25, 1},
		.left_margin	= 0x7E,
		.right_margin	= 0,
		.upper_margin	= 0x16,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.flags		= 0,
	},
};

/* venc dv preset timings */
static struct vpbe_enc_mode_info vbpe_enc_preset_timings[] = {
	{
		.name		= "480p59_94",
		.timings_type	= VPBE_ENC_DV_PRESET,
		.timings	= {V4L2_DV_480P59_94},
		.interlaced	= 0,
		.xres		= 720,
		.yres		= 480,
		.aspect		= {1, 1},
		.fps		= {5994, 100},
		.left_margin	= 0x8F,
		.right_margin	= 0,
		.upper_margin	= 0x2D,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.flags		= 0,
	},
	{
		.name		= "576p50",
		.timings_type	= VPBE_ENC_DV_PRESET,
		.timings	= {V4L2_DV_576P50},
		.interlaced	= 0,
		.xres		= 720,
		.yres		= 576,
		.aspect		= {1, 1},
		.fps		= {50, 1},
		.left_margin	= 0x8C,
		.right_margin	= 0,
		.upper_margin	= 0x36,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.flags		= 0,
	},
	{
		.name		= "720p60",
		.timings_type	= VPBE_ENC_DV_PRESET,
		.timings	= {V4L2_DV_720P60},
		.interlaced	= 0,
		.xres		= 1280,
		.yres		= 720,
		.aspect		= {1, 1},
		.fps		= {60, 1},
		.left_margin	= 0x117,
		.right_margin	= 70,
		.upper_margin	= 38,
		.lower_margin	= 3,
		.hsync_len	= 80,
		.vsync_len	= 5,
		.flags		= 0,
	},
	{
		.name		= "1080i30",
		.timings_type	= VPBE_ENC_DV_PRESET,
		.timings	= {V4L2_DV_1080I30},
		.interlaced	= 1,
		.xres		= 1920,
		.yres		= 1080,
		.aspect		= {1, 1},
		.fps		= {30, 1},
		.left_margin	= 0xc9,
		.right_margin	= 80,
		.upper_margin	= 30,
		.lower_margin	= 3,
		.hsync_len	= 88,
		.vsync_len	= 5,
		.flags		= 0,
	},
};

/* custom timings */
static struct vpbe_enc_mode_info vbpe_enc_custom_timings[] = {
	{
		.name		= "272p60",
		.timings_type	= VPBE_ENC_CUSTOM_TIMINGS,
		.timings	= {CUSTOM_TIMING_480_272},
		.interlaced	= 0,
		.xres		= 480,
		.yres		= 272,
		.aspect		= {1, 1},
		.fps		= {60, 1},
		.left_margin	= 43,
		.right_margin	= 43,
		.upper_margin	= 12,
		.lower_margin	= 12,
		.hsync_len	= 42,
		.vsync_len	= 10,
		.flags		= 0
	},
	{
		.name		= "320x240p60",
		.timings_type	= VPBE_ENC_CUSTOM_TIMINGS,
		.timings	= {CUSTOM_TIMING_480_272},
		.interlaced	= 0,
		.xres		= 320,
		.yres		= 240,
		.aspect		= {1, 1},
		.fps		= {60, 1},
		.left_margin	= 68,
		.right_margin	= 68,
		.upper_margin	= 18,
		.lower_margin	= 16,
		.hsync_len	= 42,
		.vsync_len	= 10,
		.flags		= 0
	},
};

/*
 * The outputs available from VPBE + ecnoders. Keep the
 * the order same as that of encoders. First those from venc followed by that
 * from encoders. Index in the outpuvpbe-t refers to index on a particular
 * encoder.Driver uses this index to pass it to encoder when it supports more
 * than one output. Application uses index of the array to set an output.
 */
static struct vpbe_output dm365_vpbe_outputs[] = {
	{
		.output		= {
			.index		= 0,
			.name		= "Composite",
			.type		= V4L2_OUTPUT_TYPE_ANALOG,
			.std		= VENC_STD_ALL,
			.capabilities	= V4L2_OUT_CAP_STD,
		},
		.subdev_name	= VPBE_VENC_SUBDEV_NAME,
		.default_mode	= "ntsc",
		.num_modes	= ARRAY_SIZE(vbpe_enc_std_timings),
		.modes		= vbpe_enc_std_timings,
		.if_params	= V4L2_MBUS_FMT_FIXED,
	},
	{
		.output		= {
			.index		= 1,
			.name		= "Component",
			.type		= V4L2_OUTPUT_TYPE_ANALOG,
			.capabilities	= V4L2_OUT_CAP_PRESETS,
		},
		.subdev_name	= VPBE_VENC_SUBDEV_NAME,
		.default_mode	= "480p59_94",
		.num_modes	= ARRAY_SIZE(vbpe_enc_preset_timings),
		.modes		= vbpe_enc_preset_timings,
		.if_params	= V4L2_MBUS_FMT_FIXED,
	},
	{
		.output         = {
			.index          = 2,
			.name           = "Lcdout",
			.type           = V4L2_OUTPUT_TYPE_MODULATOR,
			.capabilities   = V4L2_OUT_CAP_CUSTOM_TIMINGS,
		},
		.subdev_name    = VPBE_VENC_SUBDEV_NAME,
		.default_mode   = "320x240p60",
		.num_modes      = ARRAY_SIZE(vbpe_enc_custom_timings),
		.modes          = vbpe_enc_custom_timings,
		.if_params      = V4L2_MBUS_FMT_RGB565_2X8_BE,
	},
};

#if defined(CONFIG_VIDEO_THS7303) || defined(CONFIG_VIDEO_THS7303_MODULE)
/*
 * Amplifiers on the board
 */
static struct amp_config_info vpbe_amp = {
	.module_name	= "ths7303",
	.is_i2c		= 1,
	.board_info	= {
		I2C_BOARD_INFO("ths7303", 0x2c)
	}
};
#endif

static struct vpbe_display_config vpbe_display_cfg = {
	.module_name	= "dm365-vpbe-display",
	.i2c_adapter_id	= 1,
#if defined(CONFIG_VIDEO_THS7303) || defined(CONFIG_VIDEO_THS7303_MODULE)
	.amp		= &vpbe_amp,
#endif
	.osd		= {
		.module_name	= VPBE_OSD_SUBDEV_NAME,
	},
	.venc		= {
		.module_name	= VPBE_VENC_SUBDEV_NAME,
	},
	.num_outputs	= ARRAY_SIZE(dm365_vpbe_outputs),
	.outputs	= dm365_vpbe_outputs,
};

/*Need to review if this is necessary*/
static struct davinci_mmc_config dm365leopard_mmc_config = {
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm365leopard_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365leopard_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static void dm365leopard_usb_configure(void)
{
	davinci_cfg_reg(DM365_GPIO66);
	gpio_request(66, "usb");
	gpio_direction_output(66, 0);
	davinci_setup_usb(500, 8);
}

static void dm365leopard_tlv320aic3x_configure(void)
{
	/*
	* CLKOUT1 pin is multiplexed with GPIO35 and SPI4
	* Further details are available at the DM365 ARM
	* Subsystem Users Guide(sprufg5.pdf) pages 118, 127 - 129
	*/
	struct clk *clkout1_clk;

	davinci_cfg_reg(DM365_CLKOUT1);

	clkout1_clk = clk_get(NULL, "clkout1");
	if (IS_ERR(clkout1_clk))
		return;
	clk_enable(clkout1_clk);

	/*
	* Configure CLKOUT1 OBSCLK registers
	*/

	/* (reg OCSEL) Setting OBSCLK source with Oscillator divider output enable */
	__raw_writel(0x0,IO_ADDRESS(0x01C40C00 + 0x104));

	/* (reg OSCDIV1) Setting the Oscillator divider enable with a divider ratio of 1 */
	__raw_writel(0x8000,IO_ADDRESS(0x01C40C00 + 0x124));

	/* (reg CKEN) Setting the OBSCLK clock enable */
	__raw_writel(0x02,IO_ADDRESS(0x01C40C00 + 0x148));

}

static void dm365leopard_lcdout_configure(void)
{
	/*
	 * R/G/B 0 and 1 pins are multiplexed with GPIOs
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_VOUT_B0);
	davinci_cfg_reg(DM365_VOUT_B1);
	davinci_cfg_reg(DM365_VOUT_B2);
	davinci_cfg_reg(DM365_VOUT_R0);
	davinci_cfg_reg(DM365_VOUT_R1);
	davinci_cfg_reg(DM365_VOUT_R2);
	davinci_cfg_reg(DM365_VOUT_G0);
	davinci_cfg_reg(DM365_VOUT_G1);
	davinci_cfg_reg(DM365_VOUT_COUTL_EN);
	davinci_cfg_reg(DM365_VOUT_COUTH_EN);
	davinci_cfg_reg(DM365_VOUT_LCD_OE);
	davinci_cfg_reg(DM365_VOUT_HVSYNC);
}

static void __init leopard_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/*
 * Buttons gpio dm365(22-25), dm355(25-28)
 */
static struct gpio_keys_button leo_j4_button[] = {
		{ .desc = "Key enter",
				.gpio = GPIO(22),
				.active_low = 0,
				.code = KEY_ENTER,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key menu",
				.gpio = GPIO(23),
				.active_low = 0,
				.code = KEY_MENU,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key up",
				.gpio = GPIO(24),
				.active_low = 0,
				.code = KEY_UP,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key down",
				.gpio = GPIO(25),
				.active_low = 0,
				.code = KEY_DOWN,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
};

static struct gpio_keys_platform_data stk_j1_button_data = {
		.buttons = leo_j4_button,
		.nbuttons = ARRAY_SIZE(leo_j4_button),
		.rep =1, /* enable input subsystem auto repeat */
};

static struct platform_device gpio_keys_dev = {
		.name = "gpio-keys",
		.id = 0,
		.dev = {
				.platform_data = &stk_j1_button_data,
		}
};


static void dm365leopard_keys_configure(void)
{

	davinci_cfg_reg(DM365_GPIO22);
	davinci_cfg_reg(DM365_GPIO23);
	davinci_cfg_reg(DM365_GPIO24);
	davinci_cfg_reg(DM365_GPIO25);
	platform_device_register(&gpio_keys_dev);
}
#endif


/* only for dm386, see board-dm365-evm.c from more details */
void enable_lcd(void)
{
	/* Turn on LCD backlight for DM368 */
//	if (cpu_is_davinci_dm368()) {
//		u8 resets;
//
//		resets = __raw_readb(cpld + CPLD_RESETS);
//		/* Configure 9.25MHz clock to LCD */
//		resets |= BIT(7);
//		__raw_writeb(resets, cpld + CPLD_RESETS);
//
//		/* CPLD_CONN_GIO17 is level high */
//		__raw_writeb(0xff, cpld + CPLD_CCD_IO1);
//
//		/* CPLD_CONN_GIO17 is an output */
//		__raw_writeb(0xfb, cpld + CPLD_CCD_DIR1);
//	}
}
EXPORT_SYMBOL(enable_lcd);

/* only for dm386, see board-dm365-evm.c from more details */
void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm365_leopard_map_io(void)
{
	/* setup configuration for vpbe devices */
	dm365_set_vpbe_display_config(&vpbe_display_cfg);
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);

	dm365_init();
}

static __init void dm365_leopard_init(void)
{
	leopard_init_i2c();
	davinci_serial_init(&uart_config);

	dm365leopard_emac_configure();
	dm365leopard_usb_configure();
	dm365leopard_mmc_configure();
	dm365leopard_lcdout_configure();

	davinci_setup_mmc(0, &dm365leopard_mmc_config);

	dm365_init_asp(&dm365_leopard_snd_data);
	dm365_init_rtc();
	dm365leopard_tlv320aic3x_configure();
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	dm365leopard_keys_configure();
#endif
}

static __init void dm365_leopard_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DM365_LEOPARD, "DM365 Leopard")
/*
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc, */
	.boot_params	= (0x80000100),
	.map_io		= dm365_leopard_map_io,
	.init_irq	= dm365_leopard_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_leopard_init,
MACHINE_END

