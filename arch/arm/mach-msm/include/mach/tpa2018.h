/* arch/arm/mach-msm/include/mach/qdsp6/tpa2018.h
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */

#ifndef __LINUX_TPA2018_H
#define __LINUX_TPA2018_H
#endif

#include <linux/ioctl.h>

#define TPA2018_IOCTL_MAGIC 'f'
#define IOC_MAXNR	10
#define SPK_AMP_EN 27

#define TPA2018_SET_FIXED_GAIN	_IO(TPA2018_IOCTL_MAGIC, 1)
#define TPA2018_SET_STREAM_TYPE	_IO(TPA2018_IOCTL_MAGIC, 2)
#define TPA2018_OPEN	_IO(TPA2018_IOCTL_MAGIC, 3)
#define TPA2018_CLOSE	_IO(TPA2018_IOCTL_MAGIC, 4)
#define TPA2018_FM_HS_VOLUME_CONTROL   _IO(TPA2018_IOCTL_MAGIC, 5)
#define TPA2018_FM_SPK_VOLUME_CONTROL   _IO(TPA2018_IOCTL_MAGIC, 6)
#define TPA2018_FM_RESET_VOLUME	_IO(TPA2018_IOCTL_MAGIC, 7)
#define TPA2018_GET_HEADSET_STATUS   _IO(TPA2018_IOCTL_MAGIC, 8)
#define TPA2018_SET_SPEAKER_SWITCH   _IO(TPA2018_IOCTL_MAGIC, 9)
#define TPA2018_GET_FM_VOLUME   _IO(TPA2018_IOCTL_MAGIC, 10)

extern int tpa2018_set_control(int commad, int regiter, int value);
extern int tpa2018_software_shutdown(int command);
extern int tpa2018_headset_switch(int command);
extern int tpa2018_set_headset_state(bool en);
extern bool tpa2018_get_headset_state(void);
extern int spkr_amp(int command);
extern void set_adie_flag(int flag);
extern int get_adie_flag(void);
extern int tpa2018_mute(int command);

enum {
	TPA2051_VMBYPASS  = 1U << 0,
	TPA2051_SPKR_EN   = 1U << 1,
	TPA2051_HS_EN     = 3U << 2,
	TPA2051_SWS       = 1U << 4,
	TPA2051_LIMI_EN   = 1U << 6,
	TPA2051_LIMI_SEL  = 1U << 7,

	TPA2051_MUTE      = 7U << 5,
	TPA2051_HS        = 3U << 5,

	TPA2051_SPK_GAIN  = 1U << 7,
	TPA2051_HS_ODB    = 1U << 6,
};
