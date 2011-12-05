/*
 * Copyright (c) 2009 ACER, INC.
 * Author: Robert CH Chou <Robert_CH_Chou@acer.com.tw>
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

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#define A3_CAM_AUTOFCS  34
#define A3_POWER_KEY    35
#define A3_CAM_CAPTURE  38
#define A3_HOME_KEY     40
#define A3_VOLUME_DOWN  42
#define A3_VOLUME_UP    41
#define A3_NULL_KEY1    19 /* For HDMI AP & Simple Test */
#define A3_NULL_KEY2    20 /* For HDMI AP */

static struct gpio_event_direct_entry a3_keypad_map[] = {
	{ A3_POWER_KEY,              KEY_POWER           },
	{ A3_VOLUME_UP,              KEY_VOLUMEUP        },
	{ A3_VOLUME_DOWN,            KEY_VOLUMEDOWN      },
	{ A3_CAM_CAPTURE,            KEY_CAMERA          },
	{ A3_CAM_AUTOFCS,            211                 },
	{ A3_HOME_KEY,               KEY_HOME            },
	{ A3_NULL_KEY1,              KEY_SWITCHVIDEOMODE },
	{ A3_NULL_KEY2,              KEY_KBDILLUMTOGGLE  },
};

#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
#error "input_devs"
static int keypad_gpio_event_input_func(struct input_dev *input_dev,
		struct gpio_event_info *info,
		void **data, int func);
#endif

static struct gpio_event_input_info a3_direct_keypad_info = {
#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
	.info.func	= keypad_gpio_event_input_func,
#else
	.info.func	= gpio_event_input_func,
#endif
	.flags		= 0,
	.type		= EV_KEY,
	.keymap		= a3_keypad_map,
        .debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(a3_keypad_map)
};

static struct gpio_event_info *a3_keypad_info[] = {
	&a3_direct_keypad_info.info,
};

static struct gpio_event_platform_data a3_keypad_data = {
	.name		= "a3-gpio-keypad",
	.info		= a3_keypad_info,
	.info_count	= ARRAY_SIZE(a3_keypad_info)
};

struct platform_device a3_keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &a3_keypad_data,
	},
};

#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
/* This is used by rpc_server_handset.c for sending keycodes */
static struct input_dev *keypad_dev;
#error "ERROR input_devs type error"
static int keypad_gpio_event_input_func(struct input_dev *input_dev,
		struct gpio_event_info *info,
		void **data, int func)
{
	int err;

	err = gpio_event_input_func(input_dev, info, data, func);

	if (func == GPIO_EVENT_FUNC_INIT && !err) {
		keypad_dev = input_dev;
	} else if (func == GPIO_EVENT_FUNC_UNINIT) {
		keypad_dev = NULL;
	}

	return err;
}

struct input_dev *msm_keypad_get_input_dev(void)
{
	return keypad_dev;
}
#endif /* defined(CONFIG_MSM_RPCSERVER_HANDSET) */

static int __init a3_init_keypad(void)
{
	return platform_device_register(&a3_keypad_device);
}

device_initcall(a3_init_keypad);
