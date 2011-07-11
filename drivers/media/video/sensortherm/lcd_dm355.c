#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <video/davinci_vpbe.h>
#include <linux/platform_device.h>

#include "../davinci/dm355_ccdc_regs.h"
#include "../../../char/dm355_ipipe_hw.h"

#include <mach/hardware.h>
#include <mach/io.h>

#include <asm/io.h>

extern u32 venc_reg_in(u32 offset);

//#define READ_REG(x) (printk( KERN_INFO "%s\t 0x%08x 0x%08x\n", #x, x, venc_reg_in(x)))
#define BASE_REGISTER_START_ADDR		0x01c70000
#define BASE_REGISTER_END_ADDR			0x01c71404
#define BASE_REGISTER_SIZE				(BASE_REGISTER_END_ADDR-BASE_REGISTER_START_ADDR)

#define OFFSET_BASE_REGISTER_VPSSCLK	0x0000
#define OFFSET_BASE_REGISTER_H3A		0x0080
#define OFFSET_BASE_REGISTER_IPIPEIF	0x0100
#define OFFSET_BASE_REGISTER_OSD		0x0200
#define OFFSET_BASE_REGISTER_VENC		0x0400
#define OFFSET_BASE_REGISTER_CCDC		0x0600
#define OFFSET_BASE_REGISTER_VPSSBL		0x0800
#define OFFSET_BASE_REGISTER_IPIPE		0x1000

#define base_addr  IO_ADDRESS(BASE_REGISTER_START_ADDR)
u32 base_offset = 0;

static inline u32 regr(u32 offset) {
	return __raw_readl(base_addr + base_offset + offset);
}

static inline void regw(u32 offset, u32 val) {
	__raw_writel(val, base_addr + base_offset + offset);
}

#define READ_REG(x) (printk( KERN_INFO "%20s\t 0x%08x 0x%08x\n", #x, x, regr(x)))
#define WRITE_REG(x,y) (printk( KERN_INFO "%20s\t 0x%08x 0x%08x\n", #x, x, regw((x), (y))))

#define SET_BASE_OFFSET(x) (printk( KERN_INFO "\n%s\t 0x%08x\n\n", #x, base_offset=x))

static void print_regs(void) {




SET_BASE_OFFSET(OFFSET_BASE_REGISTER_VENC);

//FIXME
regw(VENC_DCLKPTN0,0x3);
regw(VENC_DCLKCTL,0x3);
regw(VENC_LCDOUT,0x0);
//	dispc_reg_out(VENC_VIDCTL, 0x2010);

READ_REG(VENC_VMOD);
READ_REG(VENC_VIDCTL);
READ_REG(VENC_VDPRO);
READ_REG(VENC_SYNCCTL);
READ_REG(VENC_HSPLS);
READ_REG(VENC_VSPLS);
READ_REG(VENC_HINT);
READ_REG(VENC_HSTART);
READ_REG(VENC_HVALID);
READ_REG(VENC_VINT);
READ_REG(VENC_VSTART);
READ_REG(VENC_VVALID);
READ_REG(VENC_HSDLY);
READ_REG(VENC_VSDLY);
READ_REG(VENC_YCCCTL);
READ_REG(VENC_RGBCTL);
READ_REG(VENC_RGBCLP);
READ_REG(VENC_LINECTL);
READ_REG(VENC_CULLLINE);
READ_REG(VENC_LCDOUT);
READ_REG(VENC_BRTS);
READ_REG(VENC_BRTW);
READ_REG(VENC_ACCTL);
READ_REG(VENC_PWMP);
READ_REG(VENC_PWMW);
READ_REG(VENC_DCLKCTL);
READ_REG(VENC_DCLKPTN0);
READ_REG(VENC_DCLKPTN1);
READ_REG(VENC_DCLKPTN2);
READ_REG(VENC_DCLKPTN3);
READ_REG(VENC_DCLKPTN0A);
READ_REG(VENC_DCLKPTN1A);
READ_REG(VENC_DCLKPTN2A);
READ_REG(VENC_DCLKPTN3A);
READ_REG(VENC_DCLKHS);
READ_REG(VENC_DCLKHSA);
READ_REG(VENC_DCLKHR);
READ_REG(VENC_DCLKVS);
READ_REG(VENC_DCLKVR);
READ_REG(VENC_CAPCTL);
READ_REG(VENC_CAPDO);
READ_REG(VENC_CAPDE);
READ_REG(VENC_ATR0);
READ_REG(VENC_ATR1);
READ_REG(VENC_ATR2);
READ_REG(VENC_VSTAT);
READ_REG(VENC_RAMADR);
READ_REG(VENC_RAMPORT);
READ_REG(VENC_DACTST);
READ_REG(VENC_YCOLVL);
READ_REG(VENC_SCPROG);
READ_REG(VENC_CVBS);
READ_REG(VENC_CMPNT);
READ_REG(VENC_ETMG0);
READ_REG(VENC_ETMG1);
READ_REG(VENC_ETMG2);
READ_REG(VENC_ETMG3);
READ_REG(VENC_DACSEL);
READ_REG(VENC_ARGBX0);
READ_REG(VENC_ARGBX1);
READ_REG(VENC_ARGBX2);
READ_REG(VENC_ARGBX3);
READ_REG(VENC_ARGBX4);
READ_REG(VENC_DRGBX0);
READ_REG(VENC_DRGBX1);
READ_REG(VENC_DRGBX2);
READ_REG(VENC_DRGBX3);
READ_REG(VENC_DRGBX4);
READ_REG(VENC_VSTARTA);
READ_REG(VENC_OSDCLK0);
READ_REG(VENC_OSDCLK1);
READ_REG(VENC_HVLDCL0);
READ_REG(VENC_HVLDCL1);
READ_REG(VENC_OSDHADV);
READ_REG(VENC_CLKCTL);
READ_REG(VENC_GAMCTL);
READ_REG(VENC_XHINTVL);


}

static int __init mod_init(void) {
	int ret = 0;

	printk(KERN_DEBUG "mod_init\n");
	/*
	 ret = set_base_addr();
	 if(ret != 0)
	 return ret;
	 */
	print_regs();
	return ret;
}

static void __exit mod_exit(void) {
	printk(KERN_DEBUG "mod_exit\n");
	//release_mem_region(BASE_REGISTER_START_ADDR, BASE_REGISTER_SIZE);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("Write davinci register for LCD");
MODULE_AUTHOR("Klaus Schwarzkopf <schwarzkopf@sensortherm.de>");
MODULE_LICENSE("GPL v2");
