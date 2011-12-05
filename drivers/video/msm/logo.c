/* drivers/video/msm/logo.c
 *
 * Show Logo in RLE 565 format
 *
 * Copyright (C) 2008 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#include "lcdc_samsung.h"

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#ifdef CONFIG_MACH_ACER_A3
#define fb_size(fb, bpp) ((fb)->var.xres * (fb)->var.yres * (bpp / 8))
#else
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)
#endif

#ifdef CONFIG_MACH_ACER_A3
struct WORD16{
	unsigned short pixel;
};

struct WORD24{
	unsigned char B;
	unsigned char G;
	unsigned char R;
};

struct WORD32{
	unsigned char R;
	unsigned char G;
	unsigned char B;
	unsigned char A;
};

#define DEF_FB_PIXEL(bpp, name) struct WORD##bpp name

struct FB_PIXEL{
	unsigned char *data;
	int size;
};
#endif

static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}

#ifdef CONFIG_MACH_ACER_A3
static void memset_fb_bpp(struct FB_PIXEL *bit, unsigned short _val)
{
	unsigned char val[4];
	unsigned char *bit_ptr = NULL;
	int i = 0;

	switch(bit->size)
	{
		case 4:
		{
			DEF_FB_PIXEL(32, *ptr) = (struct WORD32 *)val;
			DEF_FB_PIXEL(32, temp) = { (char)(((_val>>11)&0x1F) << 3), (char)(((_val>>5) &0x3F) << 2), (char)(((_val>>0) &0x1F) << 3), (char)(0xff) };
			*ptr = temp;
			break;
		}

		case 3:
		{
			DEF_FB_PIXEL(24, *ptr) = (struct WORD24 *)val;
			DEF_FB_PIXEL(24, temp) = { (char)(((_val>>11)&0x1F) << 3), (char)(((_val>>5) &0x3F) << 2), (char)(((_val>>0) &0x1F) << 3) };
			*ptr = temp;
			break;
		}

		case 2:
		{
			DEF_FB_PIXEL(16, *ptr) = (struct WORD16 *)val;
			DEF_FB_PIXEL(16, temp) = { _val };
			*ptr = temp;
			break;
		}

		default:
			return;
	}

	bit_ptr = bit->data;
	for (i = 0; i < bit->size; i++)
	{
		*bit_ptr++ = val[i];
	}
}
#endif

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
int load_565rle_image(char *filename)
{
	struct fb_info *info;
	int fd, err = 0;
	unsigned count, max;
	unsigned short *data, *bits, *ptr;
	int a = 0;

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
	bits = (unsigned short *)(info->screen_base);
	while (count > 3) {
		unsigned n = ptr[0];
		if (n > max)
			break;
#ifdef CONFIG_MACH_ACER_A3
		while (n) {
			memset16(bits, ptr[1], 2);
			a++;
			if (a == OS_XRES) {
				a = 0;
				bits += (FB_XRES - OS_XRES);
			}
			n--;
			bits++;
		}
#else
		memset16(bits, ptr[1], n << 1);
#endif
		bits += n;
		max -= n;
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle_image);

int load_565rle2fb_image(char *filename)
{
	struct fb_info *info;
	int fd, err = 0;
	unsigned count, max;
	unsigned short *data, *ptr;
	struct FB_PIXEL bit;
	int a = 0;
	int res = 0;

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}
	if (info->fbops->fb_open) {
		res = info->fbops->fb_open(info,1);
		if (res)
			printk(KERN_WARNING "%s: open framebuffer fail\n", __func__);
	}
	bit.size = info->var.bits_per_pixel/8;

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
	bit.data = (unsigned char *)info->screen_base;
	while (count > 3) {
		unsigned n = ptr[0];
		if (n > max)
			break;
		while(n) {
			memset_fb_bpp(&bit, ptr[1]);
			a += 1;
			if(a == OS_XRES) {
				a = 0;
				bit.data += (OS_XRES-OS_XRES) * bit.size;
			}
			bit.data += 1 * bit.size;
			n -= 1;
		}
		bit.data += n * bit.size;
		max -= n;
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle2fb_image);

