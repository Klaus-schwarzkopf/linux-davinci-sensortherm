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
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

static struct snd_platform_data dm365_leopard_snd_data = {
	.asp_chan_q = EVENTQ_3,
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

};

/* Set the input  for TVPxxx/MTxxxx sensors */
static int dm365leopard_setup_video_input(enum vpfe_subdev_id id)
{
	const char *label;

	switch (id) {

		case VPFE_SUBDEV_MT9P031:
			label = "HD imager-MT9P031";
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

/* only for dm386, see board-dm365-evm.c from more details */
void enable_lcd(void)
{
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
	/* setup input configuration for VPFE input devices */
	//dm365_set_vpfe_config(&vpfe_cfg);
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

