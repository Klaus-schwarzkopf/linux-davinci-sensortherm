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
#include <linux/autoconf.h>
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
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>
#include <media/tvp514x.h>
#include <media/tvp7002.h>

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE	0x04000000

static struct snd_platform_data dm365_leopard_snd_data = {
	.eventq_no = EVENTQ_3,
};

static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("ths7303", 0x2c),
	}
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
	.sda_pin        = 21,
	.scl_pin        = 20,
};

#if defined(CONFIG_SOC_CAMERA_MT9P031) || defined(CONFIG_SOC_CAMERA_MT9P031_MODULE)
/* Input available at the mt9p031 */
static struct v4l2_input mt9p031_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
	}
};
#endif

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
static struct tvp514x_platform_data tvp5146_pdata = {
       .clk_polarity = 0,
       .hs_polarity = 1,
       .vs_polarity = 1
};


#define TVP514X_STD_ALL        (V4L2_STD_NTSC | V4L2_STD_PAL)
/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
};

/*
 * this is the route info for connecting each input to decoder
 * ouput that goes to vpfe. There is a one to one correspondence
 * with tvp5146_inputs
 */
static struct vpfe_route tvp5146_routes[] = {
	{
		.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
	{
		.input = INPUT_SVIDEO_VI2C_VI1C,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};
#endif

#if defined(CONFIG_VIDEO_TVP7002) || defined(CONFIG_VIDEO_TVP7002_MODULE)
/* tvp7002 platform data, used during reset and probe operations */
static struct tvp7002_platform_data tvp7002_pdata = {
       .clk_polarity = 0,
       .hs_polarity = 0,
       .vs_polarity = 0,
       .fid_polarity = 0,
};


#define TVP7002_STD_ALL        (V4L2_STD_525P_60   | V4L2_STD_625P_50 	|\
				V4L2_STD_NTSC      | V4L2_STD_PAL   	|\
				V4L2_STD_720P_50   | V4L2_STD_720P_60 	|\
				V4L2_STD_1080I_50  | V4L2_STD_1080I_60 	|\
				V4L2_STD_1080P_50  | V4L2_STD_1080P_60)

/* Inputs available at the TVP7002 */
static struct v4l2_input tvp7002_inputs[] = {
	{
		.index = 0,
		.name = "Component",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP7002_STD_ALL,
	},
};
#endif


static struct vpfe_subdev_info vpfe_sub_devs[] = {
#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
	{
		.module_name = "tvp5146",
		.grp_id = VPFE_SUBDEV_TVP5146,
		.num_inputs = ARRAY_SIZE(tvp5146_inputs),
		.inputs = tvp5146_inputs,
		.routes = tvp5146_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type = VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5d),
			.platform_data = &tvp5146_pdata,
		},
	},
#endif
#if defined(CONFIG_VIDEO_TVP7002) || defined(CONFIG_VIDEO_TVP7002_MODULE)
	{
		.module_name = "tvp7002",
		.grp_id = VPFE_SUBDEV_TVP7002,
		.num_inputs = ARRAY_SIZE(tvp7002_inputs),
		.inputs = tvp7002_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_BT1120,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp7002", 0x5c),
			.platform_data = &tvp7002_pdata,
		},
	},
	{
		.module_name = "ths7353",
		.grp_id = VPFE_SUBDEV_TVP7002,
		.board_info = {
			I2C_BOARD_INFO("ths7353", 0x2e),
		},
	},
#endif
#if defined(CONFIG_SOC_CAMERA_MT9P031) || defined(CONFIG_SOC_CAMERA_MT9P031_MODULE)
	{
		.module_name = "mt9p031",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9P031,
		.num_inputs = ARRAY_SIZE(mt9p031_inputs),
		.inputs = mt9p031_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
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
};

static struct vpfe_config vpfe_cfg = {
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 Leopard",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
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
	setup_usb(500, 8);
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

void enable_lcd(void)
{
}
EXPORT_SYMBOL(enable_lcd);

void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);


static void __init leopard_init_i2c(void)
{
	davinci_cfg_reg(DM365_GPIO20);
	gpio_request(20, "i2c-scl");
	gpio_direction_output(20, 0);
	davinci_cfg_reg(DM365_I2C_SCL);

	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel		= 0,
	.ecc_mode		= NAND_ECC_HW,
	.ecc_bits		= 4,
};

static struct resource davinci_nand_resources[] = {
	{
		.start		= DM365_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DM365_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DM365_ASYNC_EMIF_CONTROL_BASE,
		.end		= DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device davinci_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(davinci_nand_resources),
	.resource		= davinci_nand_resources,
	.dev			= {
		.platform_data	= &davinci_nand_data,
	},
};

static struct platform_device *dm365_leopard_nand_devices[] __initdata = {
	&davinci_nand_device,
};


static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};
static void __init dm365_leopard_map_io(void)
{
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

#ifdef CONFIG_SND_DM365_AIC3X_CODEC
	dm365_init_asp(&dm365_leopard_snd_data);
	dm365leopard_tlv320aic3x_configure();
#elif defined(CONFIG_SND_DM365_VOICE_CODEC)
	dm365_init_vc(&dm365_leopard_snd_data);
#endif
	dm365_init_rtc();


	platform_add_devices(dm365_leopard_nand_devices,
		ARRAY_SIZE(dm365_leopard_nand_devices));
}

static __init void dm365_leopard_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DM365_LEOPARD, "DM365 Leopard")
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		= dm365_leopard_map_io,
	.init_irq	= dm365_leopard_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_leopard_init,
MACHINE_END

