/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef LCDC_SAMSUNG_H
#define LCDC_SAMSUNG_H

#include <linux/msm_mdp.h>

#define MSMFB_BKL_SAMSUNG              _IOW(MSMFB_IOCTL_MAGIC, 5, unsigned int)
#define MSMFB_BKLSAVE_SAMSUNG          _IOW(MSMFB_IOCTL_MAGIC, 6, unsigned int)
#define MSMFB_ISBTPLAY_SAMSUNG         _IOW(MSMFB_IOCTL_MAGIC, 7, unsigned int)

#define FB_XRES                         1280
#define FB_YRES                         800
#define OS_XRES                         480
#define OS_YRES                         800

#define VINFO_BPP                       32

int lcdc_samsung_isbtplay(unsigned long flag);
int lcdc_samsung_set_backlight(unsigned long gamma);
int lcdc_samsung_save_backlight(unsigned long gamma);

int samsung_panel_on(void);

int samsung_panel_off(void);

void panel_poweron(int bOnOff);

unsigned int get_panel_bpp(void);

#endif /* LCDC_SAMSUNG_H */
