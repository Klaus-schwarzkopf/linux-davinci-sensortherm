#
# Makefile for the video capture/playback device drivers.
#
mt9v126-objs := rw_i2c_reg.o mt9v126_core.o 

obj-$(CONFIG_SOC_CAMERA_MT9V126)	+= mt9v126.o

mt9v113-objs := rw_i2c_reg.o mt9v113_davinci.o

obj-$(CONFIG_SOC_CAMERA_MT9V113) 	+= mt9v113.o

obj-$(CONFIG_DAVINCI_READ_REGS)		+= reg_read.o

obj-$(CONFIG_DAVINCI_LCD_DM355)		+= lcd_dm355.o


