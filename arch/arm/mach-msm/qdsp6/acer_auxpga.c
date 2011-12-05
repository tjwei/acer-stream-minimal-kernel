/* arch/arm/mach-msm/qdsp6/acer_auxpga.c
 *
 * Copyright (C) 2010 Acer, Inc.
 * Author: Andyl Liu <Andyl_Liu@acer.com.tw>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <asm/uaccess.h>
#include <mach/board.h>
#include <mach/msm_rpcrouter.h>

#include <mach/acer_auxpga.h>

// Enable log or not
#if 0
#define ACER_DBG(fmt, arg...) printk(KERN_INFO "[AUXPGA]: %s: " fmt "\n", __FUNCTION__, ## arg)
#else
#define ACER_DBG(fmt, arg...) do {} while (0)
#endif

#define AUXPGA_DRIVER_NAME "acer-auxpga"

// AUXPGA RPC Definition
#define MVSPROG                                 0x30000014
#define MVSVERS                                 0xcf2ba98e
#define ONCRPC_MVS_AUXPGA_GAIN_CONTROL_PROC     16

// Module function
static int acer_auxpga_probe(struct platform_device *pdev);
static int acer_auxpga_remove(struct platform_device *pdev);
// Fops function
static int acer_auxpga_open(struct inode *inode, struct file *file);
static int acer_auxpga_close(struct inode *inode, struct file *file);
static int acer_auxpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

// set AUXPGA volume
static void set_auxpga_volume(u32 index);

// work queue function
static struct work_struct work_set_auxpga;
static void set_auxpga_index(struct work_struct *work);

// volume
static u32 FM_vol;
static u32 FM_index;

// volume level definition
// 0x0E is mute.
static u32 FM_volume[16] = {
	0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x09, 0x08,
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};

// status of rpc return
typedef enum {
	AUXPGA_GAIN_SETTING_PASS,
	AUXPGA_GAIN_SETTING_FAIL
} auxpga_gain_control_status_type;

// fops for auxpga
static const struct file_operations auxpga_fops = {
	.owner      = THIS_MODULE,
	.open       = acer_auxpga_open,
	.release    = acer_auxpga_close,
	.ioctl      = acer_auxpga_ioctl,
};

// miscdevice driver register
static struct miscdevice auxpga_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AUXPGA_DRIVER_NAME,
	.fops = &auxpga_fops,
};

// platform driver register
static struct platform_driver acer_auxpga_driver = {
	.probe  = acer_auxpga_probe,
	.remove = acer_auxpga_remove,
	.driver = {
		.name  = AUXPGA_DRIVER_NAME
	},
};

// platform device register
static struct platform_device acer_auxpga_device = {
	.name		= AUXPGA_DRIVER_NAME,
};

static void set_auxpga_volume(u32 index)
{
	static struct msm_rpc_endpoint *auxpga_endpoint;
	int res;
	struct set_auxpga_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!auxpga_endpoint) {
		ACER_DBG("msm_rpc_connecting!! \n");
		auxpga_endpoint = msm_rpc_connect(MVSPROG, MVSVERS, 0);

		if (IS_ERR(auxpga_endpoint)) {
			pr_err("[AUXPGA] init auxpga rpc failed!\n");
			auxpga_endpoint = 0;
			return;
		}
		ACER_DBG("msm_rpc_connect success!!\n");
	}

	req.data = cpu_to_be32(FM_volume[index]);
	ACER_DBG("send a rpc call for fm volume !! \n");
	ACER_DBG("FM_volume = 0x%x\n", FM_volume[index]);
	ACER_DBG("req.data = 0x%x\n", req.data);

	res = msm_rpc_call(auxpga_endpoint, ONCRPC_MVS_AUXPGA_GAIN_CONTROL_PROC, &req,
		sizeof(req), 5 * HZ);
	if (res == AUXPGA_GAIN_SETTING_PASS)
		ACER_DBG("rpc success !!!\n");
	else
		ACER_DBG("rpc failed !!!\n");

}

static void set_auxpga_index(struct work_struct *work)
{
	ACER_DBG("acer_auxpga.c -> set_auxpga_index !!!\n");
	set_auxpga_volume(FM_index);
}

void auxpga_set_control(u32 index)
{
	ACER_DBG("acer_auxpga.c -> auxpga_set_control !!!\n");
	FM_index = index;
	ACER_DBG("auxpga_set_control !!!  FM_index = 0x%x\n", index);
	schedule_work(&work_set_auxpga);
}
static int acer_auxpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 uparam;

	pr_debug("[AUXPGA] auxpga ioctl \n");
	if (_IOC_TYPE(cmd) != AUXPGA_IOCTL_MAGIC) {
		pr_err("[AUXPGA] IOCTL: cmd magic type error\n");
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > IOC_MAXNR) {
		pr_err("[AUXPGA] IOCTL: cmd number error\n");
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_NONE)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	if (err) {
		pr_err("[AUXPGA] IOCTL: cmd access right error\n");
		return -EFAULT;
	}

	switch(cmd){
		case AUXPGA_FM_VOLUME_CONTROL:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;

			//FM adjust volume timing is close to switching device.
			//So, we add a delay for timeing issue.
			msleep(50);

			FM_vol = uparam;
			uparam/=10;
			FM_index = uparam;
			ACER_DBG("FM_vol = %d\n", FM_index);
			schedule_work(&work_set_auxpga);

			return 0;

		case AUXPGA_GET_FM_VOLUME: {
			if (copy_to_user((void*) arg, &FM_vol, sizeof(FM_vol)))
			    return -EFAULT;
			return 0;
		}

		default:
			pr_err("[AUXPGA] IOCTL: Command not found!\n");
			return -1;
	}
}

static int acer_auxpga_open(struct inode *inode, struct file *file)
{
	pr_debug("[AUXPGA] has been opened\n");
	return 0;
}

static int acer_auxpga_close(struct inode *inode, struct file *file)
{
	pr_debug("[AUXPGA] has been closed\n");
	return 0;
}

static int acer_auxpga_probe(struct platform_device *pdev)
{
	int ret;

	INIT_WORK(&work_set_auxpga, set_auxpga_index);

	ret = misc_register(&auxpga_dev);
	if (ret) {
		pr_err("acer_auxpga_probe: auxpga_dev register failed\n");
		goto error_auxpga_dev;
	}

	FM_vol = 0;
	FM_index = 0;
	pr_info("[AUXPGA] probe done.\n");

error_auxpga_dev:
	pr_err("[AUXPGA] probe: [AUXPGA] error\n");
	return ret;
}

static int acer_auxpga_remove(struct platform_device *pdev)
{
	ACER_DBG("");
	return 0;
}

static int __init acer_auxpga_init(void)
{
	int ret;
	ACER_DBG("");
	ret = platform_driver_register(&acer_auxpga_driver);
	if (ret)
		return ret;
	return platform_device_register(&acer_auxpga_device);
}

static void __exit acer_auxpga_exit(void)
{
	platform_device_unregister(&acer_auxpga_device);
	platform_driver_unregister(&acer_auxpga_driver);
}

module_init(acer_auxpga_init);
module_exit(acer_auxpga_exit);

MODULE_AUTHOR("Andyl Liu <Andyl_Liu@acer.com.tw>");
MODULE_DESCRIPTION("ACER AUXPGA driver");
MODULE_LICENSE("GPL");
