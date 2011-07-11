/*
 * DM355 leopard board support
 *
 * Based on board-dm355-evm.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <media/mt9v126.h>
#include <media/v4l2-int-device.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/hardware.h>
#include <mach/dm355.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/nand.h>
#include <mach/mmc.h>

#include "mux.h"

#define DAVINCI_ASYNC_EMIF_CONTROL_BASE		0x01e10000
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE	0x02000000

/* NOTE:  this is geared for the standard config, with a socketed
 * 2 GByte Micron NAND (MT29F16G08FAA) using 128KB sectors.  If you
 * swap chips, maybe with a different block size, partitioning may
 * need to be changed.
 */
#define NAND_BLOCK_SIZE		SZ_128K

static struct mtd_partition davinci_nand_partitions[] = {
	{
		/* UBL (a few copies) plus U-Boot */
		.name		= "bootloader",
		.offset		= 0,
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	}, {
		/* U-Boot environment */
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1 * NAND_BLOCK_SIZE,
		.mask_flags	= 0,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "filesystem1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_512M,
		.mask_flags	= 0,
	}, {
		.name		= "filesystem2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel		= BIT(14),
	.parts			= davinci_nand_partitions,
	.nr_parts		= ARRAY_SIZE(davinci_nand_partitions),
	.ecc_mode		= NAND_ECC_HW_SYNDROME,
	.options		= NAND_USE_FLASH_BBT,
};

static struct resource davinci_nand_resources[] = {
	{
		.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DAVINCI_ASYNC_EMIF_CONTROL_BASE,
		.end		= DAVINCI_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
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

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
};

static int leopard_mmc_gpio = -EINVAL;

static void dm355leopard_mmcsd_gpios(unsigned gpio)
{
	gpio_request(gpio + 0, "mmc0_ro");
	gpio_request(gpio + 1, "mmc0_cd");
	gpio_request(gpio + 2, "mmc1_ro");
	gpio_request(gpio + 3, "mmc1_cd");

	/* we "know" these are input-only so we don't
	 * need to call gpio_direction_input()
	 */

	leopard_mmc_gpio = gpio;
}

static struct i2c_board_info dm355leopard_i2c_info[] = {
	{ I2C_BOARD_INFO("dm355leopard_msp", 0x25),
		.platform_data = dm355leopard_mmcsd_gpios,
		/* plus irq */ },
	/* { I2C_BOARD_INFO("tlv320aic3x", 0x1b), }, */
	/* { I2C_BOARD_INFO("tvp5146", 0x5d), }, */
};

static void __init leopard_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);

	gpio_request(5, "dm355leopard_msp");
	gpio_direction_input(5);
	dm355leopard_i2c_info[0].irq = gpio_to_irq(5);

	i2c_register_board_info(1, dm355leopard_i2c_info,
			ARRAY_SIZE(dm355leopard_i2c_info));
}

static struct resource dm355leopard_dm9000_rsrc[] = {
	{
		/* addr */
		.start	= 0x04000000,
		.end	= 0x04000001,
		.flags	= IORESOURCE_MEM,
	}, {
		/* data */
		.start	= 0x04000016,
		.end	= 0x04000017,
		.flags	= IORESOURCE_MEM,
	}, {
		.flags	= IORESOURCE_IRQ
			| IORESOURCE_IRQ_HIGHEDGE /* rising (active high) */,
	},
};

static struct platform_device dm355leopard_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.resource	= dm355leopard_dm9000_rsrc,
	.num_resources	= ARRAY_SIZE(dm355leopard_dm9000_rsrc),
};
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/*
 * Buttons gpio 25 - 29
 */
static struct gpio_keys_button leo_j4_button[] = {
		{ .desc = "Key enter",
				.gpio = GPIO(25),
				.active_low = 0,
				.code = KEY_ENTER,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key menu",
				.gpio = GPIO(26),
				.active_low = 0,
				.code = KEY_MENU,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key up",
				.gpio = GPIO(27),
				.active_low = 0,
				.code = KEY_UP,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key down",
				.gpio = GPIO(28),
				.active_low = 0,
				.code = KEY_DOWN,
				.type = EV_KEY,
				.debounce_interval = 20,},/* debounce ticks interval in msecs */
		{ .desc = "Key Fire",
				.gpio = GPIO(29),
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

static void setup_gpio_keys(void){
	
	davinci_cfg_reg(DM355_GPIO_25_30); /* 	configure cpio 25-30 */
        platform_device_register(&gpio_keys_dev);
}

#endif

static struct platform_device *davinci_leopard_devices[] __initdata = {
	&dm355leopard_dm9000,
	&davinci_nand_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static struct v4l2_input mt9v126_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
	}
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = MT9V126_MODULE_NAME,
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9V126,
		.num_inputs = ARRAY_SIZE(mt9v126_inputs),
		.inputs = mt9v126_inputs,
		.ccdc_if_params = {
			//.if_type = VPFE_RAW_BAYER,
			.if_type = VPFE_BT656,
			//.if_type = VPFE_YCBCR_SYNC_8,
            //.if_type = VPFE_BT656_10BIT,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
            //.hdpol = VPFE_PINPOL_NEGATIVE,
			//.vdpol = VPFE_PINPOL_NEGATIVE,
		},
		.board_info = {
			I2C_BOARD_INFO(MT9V126_MODULE_NAME, MT9V126_I2C_ADDR),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	},
};

static struct vpfe_config vpfe_cfg = {
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM355 Leopardboard",
	.ccdc = "DM355 CCDC",
	.num_clocks = 2,
	.clocks = {"vpss_master", "vpss_slave"},
};

static void __init dm355_leopard_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm355_set_vpfe_config(&vpfe_cfg);
	dm355_init();
}

static int dm355leopard_mmc_get_cd(int module)
{
	if (!gpio_is_valid(leopard_mmc_gpio))
		return -ENXIO;
	/* low == card present */
	return !gpio_get_value_cansleep(leopard_mmc_gpio + 2 * module + 1);
}

static int dm355leopard_mmc_get_ro(int module)
{
	if (!gpio_is_valid(leopard_mmc_gpio))
		return -ENXIO;
	/* high == card's write protect switch active */
	return gpio_get_value_cansleep(leopard_mmc_gpio + 2 * module + 0);
}

static struct davinci_mmc_config dm355leopard_mmc_config = {
	.get_cd		= dm355leopard_mmc_get_cd,
	.get_ro		= dm355leopard_mmc_get_ro,
	.wires		= 4,
	.max_freq       = 50000000,
	.caps           = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
};

/* Don't connect anything to J10 unless you're only using USB host
 * mode *and* have to do so with some kind of gender-bender.  If
 * you have proper Mini-B or Mini-A cables (or Mini-A adapters)
 * the ID pin won't need any help.
 */
#ifdef CONFIG_USB_MUSB_PERIPHERAL
#define USB_ID_VALUE	0	/* ID pulled high; *should* float */
#else
#define USB_ID_VALUE	1	/* ID pulled low */
#endif

static struct spi_eeprom at25640a = {
	.byte_len	= SZ_64K / 8,
	.name		= "at25640a",
	.page_size	= 32,
	.flags		= EE_ADDR2,
};

static struct spi_board_info dm355_leopard_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640a,
		.max_speed_hz	= 10 * 1000 * 1000,	/* at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};


static __init void dm355_leopard_init(void)
{
	struct clk *aemif;

	gpio_request(9, "dm9000");
	gpio_direction_input(9);
	dm355leopard_dm9000_rsrc[2].start = gpio_to_irq(9);

	aemif = clk_get(&dm355leopard_dm9000.dev, "aemif");
	if (IS_ERR(aemif))
		WARN("%s: unable to get AEMIF clock\n", __func__);
	else
		clk_enable(aemif);

	platform_add_devices(davinci_leopard_devices,
			     ARRAY_SIZE(davinci_leopard_devices));
	leopard_init_i2c();
	davinci_serial_init(&uart_config);

	/* NOTE:  NAND flash timings set by the UBL are slower than
	 * needed by MT29F16G08FAA chips ... EMIF.A1CR is 0x40400204
	 * but could be 0x0400008c for about 25% faster page reads.
	 */

	gpio_request(2, "usb_id_toggle");
	gpio_direction_output(2, USB_ID_VALUE);
	/* irlml6401 switches over 1A in under 8 msec */
	setup_usb(500, 8);

	/* setup gpio keys */ 
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	setup_gpio_keys();

#endif


	davinci_setup_mmc(0, &dm355leopard_mmc_config);
	davinci_setup_mmc(1, &dm355leopard_mmc_config);

	dm355_init_spi0(BIT(0), dm355_leopard_spi_info,
			ARRAY_SIZE(dm355_leopard_spi_info));
}

static __init void dm355_leopard_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DM355_LEOPARD, "DaVinci DM355 leopard")
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (0x80000100),
	.map_io	      = dm355_leopard_map_io,
	.init_irq     = dm355_leopard_irq_init,
	.timer	      = &davinci_timer,
	.init_machine = dm355_leopard_init,
MACHINE_END
