/*
 * Copyright (c) 2009 ACER, INC.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#if defined(CONFIG_ACER_DEBUG)
#define DEBUG
#endif
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include "msm_fb.h"
/* FIXME CONFIG_FB_MDDI_CATCH_LCDC_PRISM is incorrect. */

/* FIXME IF NOT REQUEST ADIE FLAG */
#include <mach/tpa2018.h>

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
#include "mddihosti.h"
#endif

/* Backlight */
#include "lcdc_samsung.h"
#define bkl_lvl 10
/* LCD GPIOs */
#define GPIO_LCD_RST  148
/* SPI GPIOs */
#define GPIO_SPI_CLK  150
#define GPIO_SPI_DI   153
#define GPIO_SPI_CS   103

#define GPIO_PCLK     135
#define GPIO_VSYNC    136
#define GPIO_HSYNC    137
#define GPIO_VDEN     138

#define gpio_output_enable(gpio,en) gpio_configure(gpio, en==0?GPIOF_INPUT:GPIOF_DRIVE_OUTPUT)

/* Code from fastboot */
#define LCD_RST_HI         gpio_set_value(GPIO_LCD_RST,1)
#define LCD_RST_LO         gpio_set_value(GPIO_LCD_RST,0)
#define LCD_SPI_CS_HI      gpio_set_value(GPIO_SPI_CS, 1)
#define LCD_SPI_CS_LO      gpio_set_value(GPIO_SPI_CS, 0)
#define LCD_SPI_CLK_HI     gpio_set_value(GPIO_SPI_CLK,1)
#define LCD_SPI_CLK_LO     gpio_set_value(GPIO_SPI_CLK,0)
#define LCD_SPI_SET_DI(x)  gpio_set_value(GPIO_SPI_DI, x)

volatile static unsigned char driver_ic_state;
volatile static int backlight_setting_value = 5;
spinlock_t lcdc_samsung_spin_lock = SPIN_LOCK_UNLOCKED;
static struct msm_panel_info pinfo;
int bt_flag = 0;
static int bl_flag = 0;

/* The table of gamma value */
static unsigned char samsung_gamma_value[] = {
/* reg     50     70    100    130    150    160    190    200    220    250 */
/* RED */
	0x46,  0x24,  0x29,  0x2f,  0x34,  0x37,  0x38,  0x3c,  0x3e,  0x40,  0x44,
	0x45,  0x27,  0x26,  0x24,  0x23,  0x23,  0x23,  0x22,  0x21,  0x20,  0x1f,
	0x44,  0x2d,  0x2b,  0x2b,  0x2a,  0x28,  0x28,  0x27,  0x27,  0x28,  0x27,
	0x43,  0x2c,  0x2c,  0x2a,  0x29,  0x29,  0x29,  0x29,  0x29,  0x28,  0x27,
	0x42,  0x3c,  0x35,  0x30,  0x2e,  0x2d,  0x2b,  0x29,  0x28,  0x28,  0x2a,
	0x41,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,
	0x40,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
/* GREEN */
	0x56,  0x23,  0x28,  0x2e,  0x33,  0x36,  0x37,  0x3b,  0x3d,  0x3f,  0x43,
	0x55,  0x27,  0x26,  0x24,  0x23,  0x22,  0x23,  0x22,  0x20,  0x20,  0x1f,
	0x54,  0x2a,  0x29,  0x29,  0x28,  0x28,  0x27,  0x26,  0x27,  0x27,  0x26,
	0x53,  0x22,  0x25,  0x25,  0x25,  0x25,  0x25,  0x26,  0x25,  0x25,  0x24,
	0x52,  0x00,  0x00,  0x00,  0x0a,  0x0b,  0x0b,  0x10,  0x10,  0x11,  0x17,
	0x51,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
	0x50,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
/* BLUE */
	0x66,  0x31,  0x37,  0x3f,  0X46,  0x4a,  0x4b,  0x50,  0x53,  0x56,  0x5c,
	0x65,  0x24,  0x23,  0x21,  0X20,  0x1f,  0x20,  0x1f,  0x1d,  0x1c,  0x1b,
	0x64,  0x2b,  0x2a,  0x29,  0x27,  0x26,  0x25,  0x24,  0x25,  0x26,  0x24,
	0x63,  0x2c,  0x2b,  0x29,  0x28,  0x28,  0x28,  0x28,  0x27,  0x26,  0x25,
	0x62,  0x3b,  0x34,  0x2f,  0x2d,  0x2b,  0x29,  0x28,  0x28,  0x27,  0x2a,
	0x61,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,  0x3f,
	0x60,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
};

void spi_gen(unsigned char spi_data)
{

	int bit;
	unsigned char mask;

	for (bit = 7; bit >= 0; bit--) {
		mask = (unsigned char)1 << bit;
		LCD_SPI_CLK_LO;
		if (spi_data & mask) {
			LCD_SPI_SET_DI(1);
		} else {
			LCD_SPI_SET_DI(0);
		}
		LCD_SPI_CLK_HI;
	}

}

void SPI_Write_8bits(unsigned char Index,unsigned char REG_DATA)
{

	unsigned char DeviceID = 0x70;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = REG_DATA;
/* Instruction format 3SPI
 * SPI CLK: rising edge
 * MOSI: 0  1  1  1  0 ID RS RW IB7 IB6 IB5 IB4 IB3 IB2 IB1 IB0
 *       |<-start byte->|       |<-write index & instruction->|
 */
	LCD_SPI_CS_LO;

	spi_gen(DeviceID | 0x0);
	spi_gen(lcd_Index);

	LCD_SPI_CS_HI;
	LCD_SPI_CS_LO;

	spi_gen(DeviceID | 0x2);
	spi_gen(lcd_data0);

	LCD_SPI_CS_HI;

}

void SPI_Write_16bits(unsigned char Index,unsigned short REG_DATA)
{

	unsigned char DeviceID = 0x70;
	unsigned char lcd_Index = (unsigned char)(Index & 0xff);
	unsigned char lcd_data0 = (unsigned char)( (REG_DATA>>8) & 0xff);
	unsigned char lcd_data1 = (unsigned char)( REG_DATA & 0xff);

	LCD_SPI_CS_LO;

	spi_gen(DeviceID | 0x0);
	spi_gen(lcd_Index);

	LCD_SPI_CS_HI;
	LCD_SPI_CS_LO;

	spi_gen(DeviceID | 0x2);
	spi_gen(lcd_data0);

	LCD_SPI_CS_HI;
	LCD_SPI_CS_LO;

	spi_gen(DeviceID | 0x2);
	spi_gen(lcd_data1);

	LCD_SPI_CS_HI;

}

void InitializingSequence(void)
{

	unsigned char *ptr_init;
	int j;
	ptr_init = samsung_gamma_value;
	SPI_Write_8bits(0x31,0x08);	    /* SCTE set */
	SPI_Write_8bits(0x32,0x14);	    /* SCWE set */
	SPI_Write_8bits(0x30,0x02);	    /* Gateless signal */
	SPI_Write_8bits(0x27,0x01);	    /* Gateless signal */
	SPI_Write_8bits(0x12,0x08);	    /* VBP set */
	SPI_Write_8bits(0x13,0x08);	    /* VFP set */
	SPI_Write_8bits(0x15,0x00);	    /* Display con(Vsync:Active low, Hsync:Active low, DOTCLK:rising edge) */
#ifdef CONFIG_MACH_ACER_A3
	if (pinfo.bpp == 16)
		SPI_Write_8bits(0x16,0x02);
	else
		SPI_Write_8bits(0x16,0x00);
#else
	SPI_Write_8bits(0x16,0x02);	    /* Color depth set(65k) */
#endif
	SPI_Write_16bits(0xef,0xd0e8);  /* Pentile Key */

	SPI_Write_8bits(0x39,0x44);	    /* Gamma set select */
	/* Gamma table for 250 cd/m2 */
	for (j = 0; j < 21; j++) {
		SPI_Write_8bits(*ptr_init, *(ptr_init + backlight_setting_value));
		ptr_init += (bkl_lvl + 1);
	}
	bl_flag = 1;

}

void PowerOnSettingSequence(void)
{

	SPI_Write_8bits(0x17,0x22);	/* Boosting Freq	*/
	SPI_Write_8bits(0x18,0x33);	/* AMP set */
	SPI_Write_8bits(0x19,0x03);	/* Gamma Amp */
	SPI_Write_8bits(0x1A,0x01);	/* Power Boosting Rate */
	SPI_Write_8bits(0x22,0xA4);	/* Internal Logic Voltage */
	SPI_Write_8bits(0x23,0x00);	/* Power set */
	SPI_Write_8bits(0x26,0xA0);	/* Display Condition set */

}

/* SAMSUNG LCM power on/off SPI commands */
void panel_poweron(int bOnOff)
{
	unsigned long flag;
	int i;

	if (bOnOff == 1) {
		driver_ic_state = 0;
		pr_info("[SAMSUNG]panel_poweron ++ entering\n");
		/* Start Power on sequence */
		mdelay(25);
		LCD_RST_HI;
		mdelay(1);
		LCD_RST_LO;
		mdelay(1);
		LCD_RST_HI;
		mdelay(1);

		gpio_tlmm_config(GPIO_CFG(GPIO_PCLK, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_HSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VDEN, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_SPI_CLK, 0, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_SPI_DI, 0, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_SPI_CS, 0, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		for(i = 0; i < 24; i++){
			gpio_tlmm_config(GPIO_CFG(111 + i, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_6MA), GPIO_ENABLE);
		}
		LCD_SPI_CS_HI;
		LCD_SPI_CLK_HI;
		LCD_SPI_SET_DI(1);
		mdelay(20);
		/*  QCT ADIE is playing or not */

		spin_lock_irqsave(&lcdc_samsung_spin_lock, flag);
		if (get_adie_flag() == 1 || bt_flag == 1) {
			spin_unlock_irqrestore(&lcdc_samsung_spin_lock, flag);

			InitializingSequence();

			PowerOnSettingSequence();

			SPI_Write_8bits(0x1d,0xa0); /* STB off */

#ifdef CONFIG_MACH_ACER_A3
			msleep(50);
#else
			mdelay(250);
#endif

			SPI_Write_8bits(0x14,0x03); /*DISP ON */
		} else {
			InitializingSequence();

			PowerOnSettingSequence();

			SPI_Write_8bits(0x1d,0xa0); /* STB off */

#ifdef CONFIG_MACH_ACER_A3
			mdelay(50);
#else
			mdelay(250);
#endif

			SPI_Write_8bits(0x14,0x03); /*DISP ON */

			spin_unlock_irqrestore(&lcdc_samsung_spin_lock, flag);
		}
		pr_info("[SAMSUNG]panel_poweron -- leaving\n");
		driver_ic_state = 1;
	} else {
		driver_ic_state = 0;
		pr_info("[SAMSUNG]panel_poweroff ++ entering\n");
		/* assign power-off table */
		SPI_Write_8bits(0x14,0x00); /* DISP off */

#ifdef CONFIG_MACH_ACER_A3
		msleep(80);
#else
		mdelay(80);
#endif

		SPI_Write_8bits(0x1d,0xa1); /* STB ON */

#ifdef CONFIG_MACH_ACER_A3
		msleep(200);
#else
		mdelay(200);
#endif

		/* RESTB Active */
		LCD_RST_LO;
		mdelay(1);
		LCD_RST_HI;
		mdelay(1);
		LCD_RST_LO;

		pr_info("[SAMSUNG]panel_poweroff -- leaving\n");
	}

}


/* TODO Implement samsung panel on/off */
static int lcdc_samsung_panel_on(struct platform_device *pdev)
{

	pr_debug("[SAMSUNG]%s ++ entering\n", __func__);

	pr_debug("[SAMSUNG]%s -- leaving\n", __func__);
	return 0;

}

static int lcdc_samsung_panel_off(struct platform_device *pdev)
{
	int i;

	pr_debug("[SAMSUNG]%s ++ entering\n", __func__);

	panel_poweron(0);
	gpio_tlmm_config(GPIO_CFG(GPIO_PCLK, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_VSYNC, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_HSYNC, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_VDEN, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_SPI_CLK, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_SPI_DI, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_SPI_CS, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	for(i = 0; i < 24; i++){
		gpio_tlmm_config(GPIO_CFG(111 + i, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
	}

	pr_debug("[SAMSUNG]%s -- leaving\n", __func__);
	return 0;

}

int samsung_panel_on(void)
{
	pr_debug("%s ++ entering\n", __func__);

	panel_poweron(1);

	pr_debug("%s -- leaving\n", __func__);
	return 0;
}

int samsung_panel_off(void)
{
	pr_debug("%s ++ entering\n", __func__);

	panel_poweron(0);

	pr_debug("%s -- leaving\n", __func__);
	return 0;
}

static struct msm_fb_panel_data lcdc_samsung_panel_data = {
	.on = lcdc_samsung_panel_on,
	.off = lcdc_samsung_panel_off,
};

int lcdc_samsung_isbtplay(unsigned long flag)
{
	bt_flag = (int)flag;
	pr_info("[SAMSUNG]%s (flag,bt_flag) = (%ld,%d) \n", __func__, flag,bt_flag);
	return 0;
}

int lcdc_samsung_save_backlight(unsigned long gamma)
{

	pr_debug("[SAMSUNG]%s save gamma: %ld\n", __func__, gamma);
	backlight_setting_value = (int)((gamma / 28) + 1);
	return 0;

}

int lcdc_samsung_set_backlight(unsigned long gamma)
{

	unsigned char *ptr;
	int i;
	int count = 0;
	unsigned long flag;
	ptr = samsung_gamma_value;
	pr_debug("[SAMSUNG] %s lcdc_samsung_set_backlight %ld\n", __func__, gamma);
	count = (int)((gamma / 28) + 1);
	if(driver_ic_state == 1) {
		if(bl_flag == 1) {
			ptr = samsung_gamma_value;
			spin_lock_irqsave(&lcdc_samsung_spin_lock, flag);
			SPI_Write_8bits(0x39,0x42);	    /* Gamma set select */
			if(gamma == 0) {
				SPI_Write_16bits(0x28, 0x320);
				SPI_Write_16bits(0x29, 0x320);
			} else {
				for (i = 0; i < 21; i++) {
					SPI_Write_8bits(*ptr, *(ptr + count));
					ptr += (bkl_lvl + 1);
				}
			}
			SPI_Write_8bits(0x39,0x22);	    /* Gamma set select */
			spin_unlock_irqrestore(&lcdc_samsung_spin_lock, flag);
			msleep(10);
			bl_flag = 0;
		} else {
			ptr = samsung_gamma_value;
			spin_lock_irqsave(&lcdc_samsung_spin_lock, flag);
			SPI_Write_8bits(0x39,0x24);	    /* Gamma set select */
			if(gamma == 0) {
				SPI_Write_16bits(0x28, 0x320);
				SPI_Write_16bits(0x29, 0x320);
			} else {
				for (i = 0; i < 21; i++) {
					SPI_Write_8bits(*ptr, *(ptr + count));
					ptr += (bkl_lvl + 1);
				}
			}
			SPI_Write_8bits(0x39,0x44);	    /* Gamma set select */
			spin_unlock_irqrestore(&lcdc_samsung_spin_lock, flag);
			msleep(10);
			bl_flag = 1;
		}
	} else if (driver_ic_state == 0) {
		pr_debug("[SAMSUNG] %s chip is not ready\n", __func__);
	} else {
		pr_info("[SAMSUNG] %s ERROR %d\n", __func__, driver_ic_state);
	}
	return 0;

}

unsigned int get_panel_bpp(void)
{
	return pinfo.bpp;
}

static int __init lcdc_samsung_init(void)
{

	int ret;

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
	ret = msm_fb_detect_client("lcdc_samsung_wvga");
	if (ret == -ENODEV)
		return 0;

	if (ret && (mddi_get_client_id() != 0))
		return 0;
#endif
	pinfo.xres = FB_XRES;
	pinfo.yres = FB_YRES;
	pinfo.type = LCDC_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = VINFO_BPP;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 24576000; /* 33.3MHz is limited from SUMSUNG datasheet */
	pinfo.width = 48;          /* physical width 48.24 mm */
	pinfo.height = 81;         /* physical height 80.4 mm */

	pinfo.lcdc.h_back_porch =  8;
	pinfo.lcdc.h_front_porch = 8;
	pinfo.lcdc.h_pulse_width = 1;
	pinfo.lcdc.v_back_porch =  4;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 4;
	pinfo.lcdc.border_clr = 0;        /* blk */
	pinfo.lcdc.underflow_clr = 0xff;  /* blue */
	pinfo.lcdc.hsync_skew = 0;

	ret = lcdc_device_register(&pinfo, &lcdc_samsung_panel_data);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;

}

module_init(lcdc_samsung_init);
