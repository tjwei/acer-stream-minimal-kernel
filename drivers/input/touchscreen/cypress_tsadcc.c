/*
 *  CYPRESS Touch Screen Driver
 *
 *  Copyright (c) 2008 CYPRESS
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
 *
 *  Based on touchscreen code from CYPRESS Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <mach/board.h>
#include <mach/mcu.h>

#define TS_DRIVER_NAME "cypress-touch"

#define CYPRESS_TS_X_MIN           0
#define CYPRESS_TS_X_MAX           480
#define CYPRESS_TS_Y_MIN           0
#define CYPRESS_TS_Y_MAX           800

#define USEINIT                  1

static int pre_pressed=1;
extern int Check_Touch;

typedef enum
{
	ACTIVE,
	SUSPEND,
	SUSPENDING,
	RESUME,
	INIT,
} ts_status;

struct cypress_data{
	struct work_struct work;
	struct i2c_client *client;
	struct input_dev *input;
	ts_status status;
	uint8_t key_code;
	struct cypress_platform_data* platform_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static struct cypress_data *cypress_data;

#if USEINIT
static int set_mode(ts_status status)
{
	uint8_t cPointer[2] = {0};
	static uint8_t cMM_Table[8] = {0};

	switch(status){
	case INIT:
		if (8 != i2c_master_recv(cypress_data->client, cMM_Table, 8))
			goto i2c_err;

		break;
	case SUSPEND:
	  cPointer[0] = 0;
	  cPointer[1] = 2;
	  if (4 != i2c_master_send(cypress_data->client, cPointer, 4))
	    goto i2c_err;

		break;
	case RESUME:
	  gpio_set_value(33, 0);
	  msleep(20);
	  gpio_set_value(33, 1);

		break;
	default:
		break;
	}
	if (1 !=i2c_master_send(cypress_data->client, cPointer, 1))
			pr_err("[TS][%s] probe i2c send error\n",__func__);

	return 0;
i2c_err:
	pr_err("[TS] %s error (%d)\n",__func__,status);
	return -ENXIO;
}
#endif

static void cypress_work_func(struct work_struct *work)
{
	uint8_t data[31]={0};
	uint8_t kpd_data[2]={0};
	int pressed =0, pressed1=0;
	unsigned int x,y,x1,y1,width=0;

	if (31 !=i2c_master_recv(cypress_data->client, data, 31))
		pr_err("[TS][%s] probe i2c recv error\n",__func__);

	x = data[3]<<8|data[4];
	y = data[5]<<8|data[6];
	x1 = data[9]<<8|data[10];
	y1 = data[11]<<8|data[12];
	pressed = data[8]>>4;
	pressed1 = data[8]&15;
	kpd_data[0]= data[27];

	if ( data[27] > 0) {
		a3_keypad_report_key(0, kpd_data);
		pre_pressed = pressed;
	}else{
		if ( pre_pressed ) {
			if (pressed > 0)
					pressed = 1;
			if (pressed1 > 0)
					pressed1 = 1;
			if ( pressed ) {
				input_report_abs(cypress_data->input, ABS_MT_POSITION_X, x );
				input_report_abs(cypress_data->input, ABS_MT_POSITION_Y, y );
			}
			input_report_abs(cypress_data->input, ABS_MT_WIDTH_MAJOR, width);
			input_report_abs(cypress_data->input, ABS_MT_TOUCH_MAJOR, pressed );
			input_mt_sync(cypress_data->input);

			if ( pressed1 ) {
				/*pr_debug(" x = %d  y = %d\n",x,y);*/
				input_report_abs(cypress_data->input, ABS_MT_POSITION_X, x1 );
				input_report_abs(cypress_data->input, ABS_MT_POSITION_Y, y1 );
			}
			input_report_abs(cypress_data->input, ABS_MT_TOUCH_MAJOR, pressed1 );
			input_mt_sync(cypress_data->input);
			input_sync(cypress_data->input);
		}else{
			a3_keypad_report_key(0, kpd_data);
			pre_pressed = 1;
		}
	}
}

static irqreturn_t cypress_ts_interrupt(int irq, void *dev_id)
{
	disable_irq(irq);
	schedule_work(&cypress_data->work);
	enable_irq(cypress_data->client->irq);

	return IRQ_HANDLED;
}

static int __init cypress_register_input(struct input_dev *input)
{
	input->name = TS_DRIVER_NAME;
	input->id.bustype = BUS_I2C;
	input->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(ABS_MT_TOUCH_MAJOR)] = BIT_MASK(ABS_MT_TOUCH_MAJOR);

	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, CYPRESS_TS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, CYPRESS_TS_X_MIN, CYPRESS_TS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, CYPRESS_TS_Y_MIN, CYPRESS_TS_Y_MAX, 0, 0);

	return input_register_device(input);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void cypress_early_suspend(struct early_suspend *h)
{
	pr_debug("[TS] Enter %s\n",__func__);
	set_mode(SUSPEND);

}

void cypress_early_resume(struct early_suspend *h)
{
	pr_debug("[TS] Enter %s and resume Done\n",__func__);
	set_mode(RESUME);
}
#endif

static int cypress_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	cypress_data = kzalloc(sizeof(struct cypress_data),GFP_KERNEL);
	if (cypress_data == NULL)
		return -ENOMEM;

	cypress_data->client = client;
	cypress_data->platform_data = (struct cypress_platform_data*)client->dev.platform_data;

	msleep(100);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENOTSUPP;

	strlcpy(client->name, TS_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, cypress_data);
	if(set_mode(INIT)==0){
      gpio_set_value(33, 0);
      msleep(20);
      gpio_set_value(33, 1);
	    INIT_WORK(&cypress_data->work, cypress_work_func);

	    cypress_data->input = input_allocate_device();
	    if (cypress_data->input == NULL)
		      return -ENOMEM;

	    if (cypress_register_input(cypress_data->input))
		      goto set_mode_err;

	    if (client->irq) {
		      if (request_irq(client->irq, cypress_ts_interrupt, IRQF_TRIGGER_FALLING,
				      TS_DRIVER_NAME, cypress_data))
		          goto request_irq_err;
	    }
#if USEINIT
	disable_irq(cypress_data->client->irq);
	set_mode(INIT);
	enable_irq(cypress_data->client->irq);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cypress_data->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	cypress_data->early_suspend.suspend = cypress_early_suspend;
	cypress_data->early_suspend.resume = cypress_early_resume;
	register_early_suspend(&cypress_data->early_suspend);
#endif
	pr_info("[TS] probe done\n");
	return 0;
}
else{
	pr_info("[TS] probe undone\n");
	kfree(cypress_data);
	return 0;
}
request_irq_err:
	free_irq(client->irq, cypress_data);
set_mode_err:
	input_free_device(cypress_data->input);
	kfree(cypress_data);
	pr_err("[TS] probe error\n");
	return -ENOTSUPP;
}

static int cypress_remove(struct i2c_client *client)
{
	struct cypress_data *tp = i2c_get_clientdata(client);
	input_unregister_device(tp->input);
	free_irq(client->irq, tp);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif
	kfree(cypress_data);
	return 0;
}

static const struct i2c_device_id cypress_id[] = {
	{ TS_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver cypress_driver = {
	.probe		= cypress_probe,
	.remove		= cypress_remove,
	.id_table	= cypress_id,
	.driver		= {
		.name = TS_DRIVER_NAME,
	},
};

static int __init cypress_init(void)
{
	pr_debug("[TS] Enter %s \n",__func__);
	if (Check_Touch == 1)
	  return i2c_add_driver(&cypress_driver);
	else
	  return -1;
}

static void __exit cypress_exit(void)
{
	i2c_del_driver(&cypress_driver);
}

module_init(cypress_init);
module_exit(cypress_exit);

MODULE_AUTHOR("Fanso Chen <Fanso_Chen@acer.com.tw>");
MODULE_DESCRIPTION("CYPRESS driver");
MODULE_LICENSE("GPL v2");

