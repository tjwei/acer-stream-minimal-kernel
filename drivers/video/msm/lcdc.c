/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>

#include "msm_fb.h"
#include "lcdc_samsung.h"

static int lcdc_probe(struct platform_device *pdev);
static int lcdc_remove(struct platform_device *pdev);

static int lcdc_off(struct platform_device *pdev);
static int lcdc_on(struct platform_device *pdev);

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

static struct clk *mdp_lcdc_pclk_clk;
static struct clk *mdp_lcdc_pad_pclk_clk;

int mdp_lcdc_pclk_clk_rate;
int mdp_lcdc_pad_pclk_clk_rate;

static struct platform_driver lcdc_driver = {
	.probe = lcdc_probe,
	.remove = lcdc_remove,
	.suspend = NULL,
	.suspend_late = NULL,
	.resume_early = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "lcdc",
		   },
};

static struct lcdc_platform_data *lcdc_pdata;

static int lcdc_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = panel_next_off(pdev);

	clk_disable(mdp_lcdc_pclk_clk);
	clk_disable(mdp_lcdc_pad_pclk_clk);

	if (lcdc_pdata && lcdc_pdata->lcdc_power_save)
		lcdc_pdata->lcdc_power_save(0);

	if (lcdc_pdata && lcdc_pdata->lcdc_gpio_config)
		ret = lcdc_pdata->lcdc_gpio_config(0);

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "lcdc",
					PM_QOS_DEFAULT_VALUE);

	return ret;
}

static int lcdc_on(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;
	unsigned long panel_pixclock_freq , pm_qos_freq;

	mfd = platform_get_drvdata(pdev);
	panel_pixclock_freq = mfd->fbi->var.pixclock;

	if (panel_pixclock_freq > 62000000)
		/* pm_qos_freq should be in Khz */
		pm_qos_freq = panel_pixclock_freq / 1000 ;
	else
		pm_qos_freq = 62000;

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "lcdc",
						pm_qos_freq);
	mfd = platform_get_drvdata(pdev);

	clk_enable(mdp_lcdc_pclk_clk);
	clk_enable(mdp_lcdc_pad_pclk_clk);

	if (lcdc_pdata && lcdc_pdata->lcdc_power_save)
		lcdc_pdata->lcdc_power_save(1);

	if (lcdc_pdata && lcdc_pdata->lcdc_gpio_config)
		ret = lcdc_pdata->lcdc_gpio_config(1);

	clk_set_rate(mdp_lcdc_pclk_clk, mfd->fbi->var.pixclock);
	clk_set_rate(mdp_lcdc_pad_pclk_clk, mfd->fbi->var.pixclock);
	mdp_lcdc_pclk_clk_rate = clk_get_rate(mdp_lcdc_pclk_clk);
	mdp_lcdc_pad_pclk_clk_rate = clk_get_rate(mdp_lcdc_pad_pclk_clk);

	ret = panel_next_on(pdev);
	return ret;
}

static int lcdc_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;

	if (pdev->id == 0) {
		lcdc_pdata = pdev->dev.platform_data;
		return 0;
	}

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCDC;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		printk(KERN_ERR "lcdc_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = (struct msm_fb_panel_data *)mdp_dev->dev.platform_data;
	pdata->on = lcdc_on;
	pdata->off = lcdc_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;
#ifdef CONFIG_MACH_ACER_A3
	if (pdata->panel_info.bpp == 32)
		mfd->fb_imgType = MDP_XRGB_8888;
	else if (pdata->panel_info.bpp == 24)
		mfd->fb_imgType = MDP_RGB_888;
	else if (pdata->panel_info.bpp == 16)
		mfd->fb_imgType = MDP_RGB_565;
	else
		pr_err("The panel can not support %dbpp format\n", VINFO_BPP);
#else
	mfd->fb_imgType = MDP_RGB_565;
#endif

	fbi = mfd->fbi;
	fbi->var.pixclock = clk_round_rate(mdp_lcdc_pclk_clk,
					mfd->panel_info.clk_rate);
	fbi->var.left_margin = mfd->panel_info.lcdc.h_back_porch;
	fbi->var.right_margin = mfd->panel_info.lcdc.h_front_porch;
	fbi->var.upper_margin = mfd->panel_info.lcdc.v_back_porch;
	fbi->var.lower_margin = mfd->panel_info.lcdc.v_front_porch;
	fbi->var.hsync_len = mfd->panel_info.lcdc.h_pulse_width;
	fbi->var.vsync_len = mfd->panel_info.lcdc.v_pulse_width;

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);

	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto lcdc_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;
		return 0;

lcdc_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int lcdc_remove(struct platform_device *pdev)
{
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ , "lcdc");
	return 0;
}

static int lcdc_register_driver(void)
{
	return platform_driver_register(&lcdc_driver);
}

static int __init lcdc_driver_init(void)
{
	mdp_lcdc_pclk_clk = clk_get(NULL, "mdp_lcdc_pclk_clk");
	if (IS_ERR(mdp_lcdc_pclk_clk)) {
		printk(KERN_ERR "error: can't get mdp_lcdc_pclk_clk!\n");
		return IS_ERR(mdp_lcdc_pclk_clk);
	}
	mdp_lcdc_pad_pclk_clk = clk_get(NULL, "mdp_lcdc_pad_pclk_clk");
	if (IS_ERR(mdp_lcdc_pad_pclk_clk)) {
		printk(KERN_ERR "error: can't get mdp_lcdc_pad_pclk_clk!\n");
		return IS_ERR(mdp_lcdc_pad_pclk_clk);
	}
	pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ , "lcdc",
				PM_QOS_DEFAULT_VALUE);
	return lcdc_register_driver();
}

module_init(lcdc_driver_init);
