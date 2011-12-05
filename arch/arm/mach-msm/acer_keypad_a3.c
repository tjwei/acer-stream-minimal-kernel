/*
 * Copyright (c) 2009 ACER, INC.
 * Author: Shawn Tu <Shawn_Tu@acer.com.tw>
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

#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/switch.h>
#include <mach/mcu.h>

#define DEV_IOCTLID               0x20
#define IOC_MAXNR                 7
#define IOCTL_SIMPLE_TEST_ON      _IOW(DEV_IOCTLID, 1, int)
#define IOCTL_SIMPLE_TEST_OFF     _IOW(DEV_IOCTLID, 2, int)
#define IOCTL_HDMI_NEMO_AP_ON     _IOW(DEV_IOCTLID, 3, int)
#define IOCTL_HDMI_NEMO_AP_OFF    _IOW(DEV_IOCTLID, 4, int)
#define IOCTL_TEST_KEY_UP         _IOW(DEV_IOCTLID, 5, int)
#define IOCTL_TEST_KEY_DOWN       _IOW(DEV_IOCTLID, 6, int)

#define DRIVER_NAME	"a3-keypad"

static int __init a3_keypad_init(void);
static int a3_keypad_probe(struct platform_device *pdev);
static int a3_keypad_remove(struct platform_device *pdev);
static int a3_keypad_open(struct inode *inode, struct file *file);
static int a3_keypad_close(struct inode *inode, struct file *file);
static int a3_keypad_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int __init a3_keypad_register_input(struct input_dev *input);
void a3_keypad_report_key(int num, uint8_t buf[2]);
int a3_simple_test_report(void);
int a3_hdmi_nemo_ap_report(void);
static bool bsimple_test = false;
static bool bhdmi_nemo_ap = false;

static struct a3_keypad_data {
	struct switch_dev sdev;
	struct input_dev *input;
	int prekey;
} kpd_data;

/* File operation of a3_keypad device file */
static const struct file_operations a3_keypad_fops = {
	.owner     = THIS_MODULE,
	.open      = a3_keypad_open,
	.release   = a3_keypad_close,
	.ioctl     = a3_keypad_ioctl,
};

static struct miscdevice a3_keypad_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRIVER_NAME,
	.fops = &a3_keypad_fops,
};

static struct platform_driver a3_keypad_driver = {
	.probe     = a3_keypad_probe,
	.remove    = a3_keypad_remove,
	.driver    = {
		.name    = DRIVER_NAME,
		.owner   = THIS_MODULE,
	},
};

static int __init a3_keypad_init(void)
{
	int ret;

	ret = platform_driver_register(&a3_keypad_driver);
	if (ret){
		pr_err("[A3-KPD] a3_keypad_init failed! \n");
	}

	return ret;
}

static int a3_keypad_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret;

	kpd_data.prekey = 1;

	kpd_data.sdev.name = DRIVER_NAME;
	ret = switch_dev_register(&kpd_data.sdev);
	if (ret < 0)
	{
		pr_err("switch_dev fail!\n");
		goto err_switch_dev_register;
	}
	else
		pr_debug("### a3_keypad_switch_dev success register ###\n");

	err = misc_register(&a3_keypad_dev);
	if (err) {
		pr_err("avr_probe: avr_dev register failed\n");
		goto error_dev;
	}

	/* input register */
	kpd_data.input = input_allocate_device();
	if (kpd_data.input == NULL) {
		pr_err("[A3-KPD] input_allocate_device error!\n");
		return -ENOMEM;
	}

	err = a3_keypad_register_input(kpd_data.input);
	if (err < 0) {
		pr_err("[A3-KPD] register_input error\n");
		goto err_register_input_dev;
	}

	pr_info("[A3-KPD] Probe done\n");

	return 0;

error_dev:
err_register_input_dev:
	input_free_device(kpd_data.input);
err_switch_dev_register:
	pr_err("[A3-KPD] Probe error\n");

	return ret;
}

static int a3_keypad_remove(struct platform_device *pdev)
{
	input_unregister_device(kpd_data.input);
	switch_dev_unregister(&kpd_data.sdev);

	return 0;
}

static int __init a3_keypad_register_input(struct input_dev *input)
{
	input->name = DRIVER_NAME;
	input->evbit[0] = BIT_MASK(EV_SYN)|BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME);
	input->keybit[BIT_WORD(KEY_BACK)] = BIT_MASK(KEY_BACK)|BIT_MASK(KEY_MENU);
	input->keybit[BIT_WORD(KEY_SEND)] |= BIT_MASK(KEY_SEND);
	input->keybit[BIT_WORD(0xE5)] |= BIT_MASK(0xE5);
	input->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);
	input->keybit[BIT_WORD(KEY_NEXTSONG)] |= BIT_MASK(KEY_NEXTSONG);
	input->keybit[BIT_WORD(KEY_PLAYPAUSE)] |= BIT_MASK(KEY_PLAYPAUSE);
	input->keybit[BIT_WORD(KEY_PREVIOUSSONG)] |= BIT_MASK(KEY_PREVIOUSSONG);
	input->keybit[BIT_WORD(KEY_FASTFORWARD)] |= BIT_MASK(KEY_FASTFORWARD);
	input->keybit[BIT_WORD(KEY_REWIND)] |= BIT_MASK(KEY_REWIND);
	input->keybit[BIT_WORD(KEY_STOP)] |= BIT_MASK(KEY_STOP);
	input->keybit[BIT_WORD(KEY_ENTER)] |= BIT_MASK(KEY_ENTER);
	input->keybit[BIT_WORD(KEY_UP)] |= BIT_MASK(KEY_UP);
	input->keybit[BIT_WORD(KEY_DOWN)] |= BIT_MASK(KEY_DOWN);
	input->keybit[BIT_WORD(KEY_LEFT)] |= BIT_MASK(KEY_LEFT);
	input->keybit[BIT_WORD(KEY_RIGHT)] |= BIT_MASK(KEY_RIGHT);
	return input_register_device(input);
}

/* open command for A3-KPD device file	*/
static int a3_keypad_open(struct inode *inode, struct file *file)
{
	return 0;
}
/* close command for A3-KPD device file */
static int a3_keypad_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int a3_keypad_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	/* check cmd */
	if(_IOC_TYPE(cmd) != DEV_IOCTLID){
		pr_err("cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("cmd number error\n");
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("cmd access_ok error\n");
		return -EFAULT;
	}

	/* cmd mapping */
	switch(cmd){
	case IOCTL_SIMPLE_TEST_ON:
		bsimple_test = true;
		pr_info("[A3-KPD] IOCTL_bSIMPLE_TEST_ON. \n");
		break;
	case IOCTL_SIMPLE_TEST_OFF:
		bsimple_test = false;
		pr_info("[A3-KPD] IOCTL_bSIMPLE_TEST_ON. \n");
		break;
	case IOCTL_HDMI_NEMO_AP_ON:
		bhdmi_nemo_ap = true;
		pr_info("[A3-KPD] IOCTL_HDMI_NEMO_AP_ON. \n");
		break;
	case IOCTL_HDMI_NEMO_AP_OFF:
		bhdmi_nemo_ap = false;
		pr_info("[A3-KPD] IOCTL_HDMI_NEMO_AP_OFF. \n");
		break;
	case IOCTL_TEST_KEY_UP:
		input_report_key(kpd_data.input, (unsigned int)arg, 0);
		pr_info("[A3-KPD] IOCTL_TEST_KEY_UP, KEY %d UP! \n", (unsigned int)arg);
		break;
	case IOCTL_TEST_KEY_DOWN:
		input_report_key(kpd_data.input, (unsigned int)arg, 1);
		pr_info("[A3-KPD] IOCTL_TEST_KEY_DOWN, KEY %d DOWN! \n", (unsigned int)arg);
		break;
	default:
		return -1;
	}

	return 0;
}

void a3_keypad_report_key(int num, uint8_t buf[2])
{
	/* num:0 ,key_report: touch */
	/* num:1 ,key_report: mcu   */
	/* num:2 ,key_report: gpio  */
	uint8_t data_buf[2] = {0};
	int key_code = 0;

	data_buf[0] = buf[0];
	data_buf[1] = buf[1];

#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
	if(!mcu_probe_check())
		return;

	if(!screen_off_check()){
		if( data_buf[0]!=0 )
		{
			kpd_led_on();
		}
		else
		{
			kpd_led_off();
		}
	}
#endif

	switch(num){
	case 0:
		if( data_buf[0] == 0x1)
		{
			key_code = KEY_SEARCH;
		}
		else if( data_buf[0] == 0x2)
		{
			key_code = KEY_BACK;
		}
		else if((data_buf[0] == 0x3)||(data_buf[0] == 0x4))
		{
			key_code = 0xE5; /* MENU */
		}

		pr_debug("[A3-KPD] report from Touch Panel\n");
		break;
	case 1:
		if(data_buf[0] == 1)
		{
			key_code = KEY_NEXTSONG;
		}
		else if(data_buf[0] == 2)
		{
			key_code = KEY_PLAYPAUSE;
		}
		else if(data_buf[0] == 3)
		{
			key_code = KEY_PREVIOUSSONG;
		}

		pr_debug("[A3-KPD] report from MCU\n");
		break;
	case 2:
		if(data_buf[0] == KEY_FASTFORWARD)
		{
			key_code = KEY_FASTFORWARD;
		}
		else if(data_buf[0] == KEY_NEXTSONG)
		{
			key_code = KEY_NEXTSONG;
		}
		else if(data_buf[0] == KEY_PLAYPAUSE)
		{
			key_code = KEY_PLAYPAUSE;
		}
		else if(data_buf[0] == KEY_PREVIOUSSONG)
		{
			key_code = KEY_PREVIOUSSONG;
		}
		else if(data_buf[0] == KEY_REWIND)
		{
			key_code = KEY_REWIND;
		}
		else if(data_buf[0] == KEY_STOP)
		{
			key_code = KEY_STOP;
		}
		else if(data_buf[0] == KEY_ENTER)
		{
			key_code = KEY_ENTER;
		}
		else if(data_buf[0] == KEY_UP)
		{
			key_code = KEY_UP;
		}
		else if(data_buf[0] == KEY_DOWN)
		{
			key_code = KEY_DOWN;
		}
		else if(data_buf[0] == KEY_LEFT)
		{
			key_code = KEY_LEFT;
		}
		else if(data_buf[0] == KEY_RIGHT)
		{
			key_code = KEY_RIGHT;
		}
		else if(data_buf[0] == KEY_MENU)
		{
			key_code = 0xE5; /* MENU */
		}
		else if(data_buf[0] == KEY_BACK)
		{
			key_code = KEY_BACK;
		}

		pr_debug("[A3-KPD] report from HDMI CEC\n");
		break;
	case 3:
		pr_debug("[A3-KPD] report from GPIO\n");
		return;
	default:
		break;
	}

	pr_debug("%s: key_code=0x%x, pre=0x%x\n",
		__func__, key_code, kpd_data.prekey);

	if( key_code != kpd_data.prekey ){
		input_report_key(kpd_data.input, kpd_data.prekey, 0);
	}
	if( key_code ) {
		input_report_key(kpd_data.input, key_code, 1);
	}
	kpd_data.prekey = key_code;
}
EXPORT_SYMBOL(a3_keypad_report_key);

int a3_simple_test_report(void)
{
	return bsimple_test;
}
EXPORT_SYMBOL(a3_simple_test_report);

int a3_hdmi_nemo_ap_report(void)
{
	return bhdmi_nemo_ap;
}
EXPORT_SYMBOL(a3_hdmi_nemo_ap_report);

static void __exit a3_keypad_exit(void)
{
	platform_driver_unregister(&a3_keypad_driver);
}

module_init(a3_keypad_init);
module_exit(a3_keypad_exit);

MODULE_AUTHOR("Shawn Tu <Shawn_Tu@acer.com.tw>");
MODULE_DESCRIPTION("ACER Keypad");
MODULE_LICENSE("GPL");
