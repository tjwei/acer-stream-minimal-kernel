/*
 *  Atmel Touch Screen Driver
 *
 *  Copyright (c) 2008 ATMEL
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
 *
 *  Based on touchscreen code from Atmel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

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

#define TS_DRIVER_NAME       "atmel-touch"

#define ATMEL_TS_X_MIN       0
#define ATMEL_TS_X_MAX       480
#define ATMEL_TS_Y_MIN       0
#define ATMEL_TS_Y_MAX       800

#define USEINIT              1
#define USE_FS               1

#define USER_DATA_VERSION    0x0B

#define TOUCH_NUMBER         3

#define MM_TABLE_SIZE        112 // Information Block
                                 // = ID Information + Object Table*6 + Checksum Field
                                 // = 7 + 17*6 + 3
static uint8_t cMM_Table[MM_TABLE_SIZE] = {0};


typedef struct {
    uint8_t pressed;
    uint16_t xPosition;
    uint16_t yPosition;
    uint8_t size;
} touch_message;

typedef enum {
    ACTIVE,
    SUSPEND,
    SUSPENDING,
    RESUME,
    INIT,
} ts_status;

struct atmel_data {
    struct work_struct work;
    struct work_struct calib_work;
    struct i2c_client *client;
    struct input_dev *input;
    ts_status status;
    uint8_t key_code;
    struct atmel_platform_data *platform_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

static struct atmel_data *atmel_data;
int Check_Touch;

#if USE_FS
static uint8_t ts_atoi(const char *name)
{
    uint8_t val = 0;

    for ( ; ;name++) {
        pr_debug("[TS] *name : %d\n", *name);
        switch (*name) {
            case '0' ... '9':
                val = 10*val + (*name-'0');
                break;

            default:
                return val;
        }
    }
}

static ssize_t set_ts_sensitivity(struct device *device,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    uint8_t cPointer[3] = {0};
    uint16_t start_position;
    uint8_t sensitivity = 0;

    sensitivity = ts_atoi(buf);

    if (cMM_Table[2] != 0x14) {
        // pointe to TCHTHR (Byte 7 of T9 Multiple Touch)
        start_position = (((uint16_t)cMM_Table[39]<<8) | (uint16_t)cMM_Table[38]) + 7;
        cPointer[0] = (uint8_t)(start_position & 0xFF);
        cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        cPointer[2] = sensitivity;
        pr_debug("[TS] Sensitivity : %d\n", ts_atoi(buf));
        if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
            pr_err("[TS][%s] set sensitivity error\n", __func__);

        // pointe to ATCHCALSTHR (Byte 7 of T8 Acquire Configuration)
        start_position = (((uint16_t)cMM_Table[32]<<8) | (uint16_t)cMM_Table[33]) + 7;
        cPointer[0] = (uint8_t)(start_position & 0xFF);
        cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        cPointer[2] = sensitivity - 10;
        if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
            pr_err("[TS][%s] set antitouch suspend threshold error\n", __func__);

        // point to T5 Message Processor
        cPointer[0] = cMM_Table[8];
        cPointer[1] = cMM_Table[9];
        if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
            pr_err("[TS][%s] point to T5 error\n", __func__);
    }

    return count;
}

static struct device_attribute ts_attrs =
    __ATTR(sensitivity, S_IRWXUGO, NULL, set_ts_sensitivity);
#endif


static void check_chip_calibration(void)
{
    uint8_t cPointer[3] = {0};
    uint16_t start_position = 0;
    uint8_t touch_flags[82] = {0};
    uint8_t try_count=0, i=0, j=0, check_mask=0;
    uint16_t touch_count=0, antitouch_count=0;

    // set Diagnostic Debug object to Touch/Antitouch Mode
    // point to DIAGNOSTIC (Byte 5 of T6 Command Porcessor)
    start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 5;
    cPointer[0] = (uint8_t)(start_position & 0xFF);
    cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
    cPointer[2] = 0xF3;
    if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
        pr_err("[TS][%s] set debug mode error\n", __func__);

    // check if touch count is less than antitouch count
    // bytes_per_channel is 2, channels_per_page is 64
    // maximum number of X lines is 20 for touch flags and antitouch flags
    // totally 40 bytes in use

    // point to T37 Debug Diagnostic
    start_position = (((uint16_t)cMM_Table[105]<<8) | (uint16_t)cMM_Table[104]);
    cPointer[0] = (uint8_t)(start_position & 0xFF);
    cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
    if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
        pr_err("[TS][%s] point to debug data error\n", __func__);

    while (!((touch_flags[0]==0xF3) && (touch_flags[1] == 0x00))) {
        if (try_count > 100) {
            pr_info("[TS] wait for page reset time out\n");
            return;
        }
        mdelay(5);
        try_count++;
        if (2 != i2c_master_recv(atmel_data->client, touch_flags, 2))
            pr_err("[TS][%s] read mode/page error\n", __func__);
    }

    if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
        pr_err("[TS][%s] point to debug data error\n", __func__);

    if (82 != i2c_master_recv(atmel_data->client, touch_flags, 82))
        pr_err("[TS][%s] read debug data error\n", __func__);

    for (i=0; i<40; i++) {
        for (j=0; j<8; j++) {
            check_mask = 1 << j;

            if (touch_flags[2+i] & check_mask)
                touch_count++;

            if (touch_flags[2+40+i] & check_mask)
                antitouch_count++;
        }
    }

    // increment page number
    // point to DIAGNOSTIC (Byte 5 of T6 Command Porcessor)
    start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 5;
    cPointer[0] = (uint8_t)(start_position & 0xFF);
    cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
    cPointer[2] = 0x01;
    if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
        pr_err("[TS][%s] page up error\n", __func__);

    if (antitouch_count > touch_count) {
        pr_info("[TS] bad calibration, do it again\n");

        // calibrate touch sensor controller
        // point to T6 Command Processor
        // Byte 2 of T6 is CALIBRATE Field
        start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 2;
        cPointer[0] = (uint8_t)(start_position & 0xFF);
        cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        cPointer[2] = 0x01;
        if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
            pr_err("[TS][%s] calibration error\n", __func__);
    }
}


static void check_chip_delta(void)
{
    uint8_t cPointer[3] = {0};
    uint16_t start_position = 0;
    uint8_t raw_deltas[128] = {0};
    uint8_t data_index=0, page_index=0, calib_flag=0x01;
    int16_t delta_value = 0;

    // set Diagnostic Debug object to Deltas Mode
    // point to DIAGNOSTIC (Byte 5 of T6 Command Porcessor)
    start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 5;
    cPointer[0] = (uint8_t)(start_position & 0xFF);
    cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
    cPointer[2] = 0x10;
    if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
        pr_err("[TS][%s] set debug mode error\n", __func__);

    // check delta values for all channels
    // bytes_per_channel is 2, channels_per_page is 64
    // there are 19*11=209 channels, totally 3 pages
    while ((page_index<3) && (calib_flag)) {
        if (cMM_Table[2] == 0x14) {
            // read delta values for HW Version 1.4
            // point to DATA[n] (Byte 2 of T37 Debug Diagnostic)
            start_position = (((uint16_t)cMM_Table[93]<<8) | (uint16_t)cMM_Table[92]) + 2;
            cPointer[0] = (uint8_t)(start_position & 0xFF);
            cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);

        } else {
            // read delta values for HW Version 1.5, 1.6, 1.0
            // point to DATA[n] (Byte 2 of T37 Debug Diagnostic)
            start_position = (((uint16_t)cMM_Table[105]<<8) | (uint16_t)cMM_Table[104]) + 2;
            cPointer[0] = (uint8_t)(start_position & 0xFF);
            cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        }
        if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
            pr_err("[TS][%s] point to debug data error\n", __func__);
        mdelay(50);
        if (128 != i2c_master_recv(atmel_data->client, raw_deltas, 128))
            pr_err("[TS][%s] read debug data error\n", __func__);

        for (data_index=0; data_index<128; data_index+=2) {
            delta_value = (int16_t)(((uint16_t)raw_deltas[data_index+1]<<8) |
                                    (uint16_t)raw_deltas[data_index]);
            if (delta_value > 400) {
                calib_flag = 0;
                break;
            }
        }

        // increment page number
        // point to DIAGNOSTIC (Byte 5 of T6 Command Porcessor)
        start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 5;
        cPointer[0] = (uint8_t)(start_position & 0xFF);
        cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        cPointer[2] = 0x01;
        if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
            pr_err("[TS][%s] page up error\n", __func__);

        page_index++;
    }

    if (calib_flag) {
        // calibrate touch sensor controller
        // point to T6 Command Processor
        // Byte 2 of T6 is CALIBRATE Field
        start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 2;
        cPointer[0] = (uint8_t)(start_position & 0xFF);
        cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
        cPointer[2] = 0x01;
        if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
            pr_err("[TS][%s] calibration error\n", __func__);
    }
}


#if USEINIT
static int set_mode(ts_status status)
{
    uint8_t cPointer[33] = {0};
    uint16_t start_position = 0;

    switch (status) {
        case INIT:
            // read Information Block from Memory Map Structure
            // point to 0x0000
            cPointer[0] = 0;
            cPointer[1] = 0;
            if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
                goto i2c_err;

            if (MM_TABLE_SIZE != i2c_master_recv(atmel_data->client, cMM_Table, MM_TABLE_SIZE))
                goto i2c_err;

            // print ID Information
            pr_info("[TS] Family ID = 0x%02X, Variant ID = 0x%02X\n", cMM_Table[0], cMM_Table[1]);
            pr_info("[TS] Version = 0x%02X, Build = 0x%02X\n", cMM_Table[2], cMM_Table[3]);
            pr_info("[TS] Matrix X Size = %d, Matrix Y Size = %d\n", cMM_Table[4], cMM_Table[5]);
            pr_info("[TS] Object Number = %d\n", cMM_Table[6]);


            if (cMM_Table[2] == 0x14) {
                // config for HW Version 1.4
                pr_info("[TS] config for HW Version 1.4\n");

                // T7 Power Configuration
                cPointer[0] = cMM_Table[20];
                cPointer[1] = cMM_Table[21];
                cPointer[2] = 0x20;
                cPointer[3] = 0x0A;
                cPointer[4] = 0x32;
                if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                    goto i2c_err;

                // T8 Acquire Configuration
                cPointer[0] = cMM_Table[26];
                cPointer[1] = cMM_Table[27];
                cPointer[2] = 0x08;
                cPointer[3] = 0x05;
                cPointer[4] = 0x14;
                cPointer[5] = 0x14;
                cPointer[6] = 0x00;
                cPointer[7] = 0x00;
                cPointer[8] = 0x00;
                cPointer[9] = 0x00;
                if (10 != i2c_master_send(atmel_data->client, cPointer, 10))
                    goto i2c_err;

                // T9 Multiple Touch
                cPointer[0] = cMM_Table[32];
                cPointer[1] = cMM_Table[33];
                cPointer[2] = 0x83;
                cPointer[3] = 0x00;
                cPointer[4] = 0x00;
                cPointer[5] = 0x10;
                cPointer[6] = 0x0A;
                cPointer[7] = 0x00;
                cPointer[8] = 0x21;
                cPointer[9] = 0x4B;
                cPointer[10] = 0x02;
                cPointer[11] = 0x03;
                cPointer[12] = 0x00;
                cPointer[13] = 0x32;
                cPointer[14] = 0x0A;
                cPointer[15] = 0x0D;
                cPointer[16] = 0x0A;
                cPointer[17] = 0x0A;
                cPointer[18] = 0x0A;
                cPointer[19] = 0x0A;
                cPointer[20] = 0x20;
                cPointer[21] = 0x03;
                cPointer[22] = 0xE0;
                cPointer[23] = 0x01;
                cPointer[24] = 0x14;
                cPointer[25] = 0x14;
                cPointer[26] = 0x14;
                cPointer[27] = 0x14;
                cPointer[28] = 0x96;
                cPointer[29] = 0x33;
                cPointer[30] = 0x9A;
                cPointer[31] = 0x52;
                if (32 != i2c_master_send(atmel_data->client, cPointer, 32))
                    goto i2c_err;

                // T15 Key Array
                cPointer[0] = cMM_Table[38];
                cPointer[1] = cMM_Table[39];
                cPointer[2] = 0x83;
                cPointer[3] = 0x10;
                cPointer[4] = 0x0A;
                cPointer[5] = 0x03;
                cPointer[6] = 0x01;
                cPointer[7] = 0x00;
                cPointer[8] = 0x21;
                cPointer[9] = 0x1E;
                cPointer[10] = 0x02;
                if (11 != i2c_master_send(atmel_data->client, cPointer, 11))
                    goto i2c_err;

                // T19 GPIO
                cPointer[0] = cMM_Table[44];
                cPointer[1] = cMM_Table[45];
                cPointer[2] = 0x00;
                cPointer[3] = 0x00;
                cPointer[4] = 0x00;
                cPointer[5] = 0x00;
                cPointer[6] = 0x00;
                cPointer[7] = 0x00;
                cPointer[8] = 0x00;
                cPointer[9] = 0x00;
                cPointer[10] = 0x00;
                cPointer[11] = 0x00;
                cPointer[12] = 0x00;
                cPointer[13] = 0x00;
                if (14 != i2c_master_send(atmel_data->client, cPointer, 14))
                    goto i2c_err;

                // T20 Grip/Face Suppression
                cPointer[0] = cMM_Table[50];
                cPointer[1] = cMM_Table[51];
                cPointer[2] = 0x00;
                cPointer[3] = 0x64;
                cPointer[4] = 0x64;
                cPointer[5] = 0x64;
                cPointer[6] = 0x64;
                cPointer[7] = 0x00;
                cPointer[8] = 0x00;
                cPointer[9] = 0x00;
                cPointer[10] = 0x00;
                cPointer[11] = 0x00;
                cPointer[12] = 0x00;
                if (13 != i2c_master_send(atmel_data->client, cPointer, 13))
                    goto i2c_err;

                // T22 Noise Suppression
                cPointer[0] = cMM_Table[56];
                cPointer[1] = cMM_Table[57];
                cPointer[2] = 0x05;
                cPointer[3] = 0x00;
                cPointer[4] = 0x19;
                cPointer[5] = 0xE7;
                cPointer[6] = 0x04;
                cPointer[7] = 0x32;
                cPointer[8] = 0x01;
                cPointer[9] = 0x0A;
                cPointer[10] = 0x0F;
                cPointer[11] = 0x14;
                cPointer[12] = 0x19;
                cPointer[13] = 0x1E;
                cPointer[14] = 0x04;
                if (15 != i2c_master_send(atmel_data->client, cPointer, 15))
                    goto i2c_err;

                // T23 Proximity
                cPointer[0] = cMM_Table[62];
                cPointer[1] = cMM_Table[63];
                cPointer[2] = 0x00;
                cPointer[3] = 0x00;
                cPointer[4] = 0x00;
                cPointer[5] = 0x00;
                cPointer[6] = 0x00;
                cPointer[7] = 0x00;
                cPointer[8] = 0x00;
                cPointer[9] = 0x00;
                cPointer[10] = 0x00;
                cPointer[11] = 0x00;
                cPointer[12] = 0x00;
                cPointer[13] = 0x00;
                if (14 != i2c_master_send(atmel_data->client, cPointer, 14))
                    goto i2c_err;

                // T24 One-Touch Gesture
                cPointer[0] = cMM_Table[68];
                cPointer[1] = cMM_Table[69];
                cPointer[2] = 0x03;
                cPointer[3] = 0x0A;
                cPointer[4] = 0xFF;
                cPointer[5] = 0x03;
                cPointer[6] = 0x00;
                cPointer[7] = 0x64;
                cPointer[8] = 0x64;
                cPointer[9] = 0x01;
                cPointer[10] = 0x0A;
                cPointer[11] = 0x14;
                cPointer[12] = 0x28;
                cPointer[13] = 0x4B;
                cPointer[14] = 0x00;
                cPointer[15] = 0x02;
                cPointer[16] = 0x00;
                cPointer[17] = 0x64;
                cPointer[18] = 0x00;
                cPointer[19] = 0x19;
                cPointer[20] = 0x00;
                if (21 != i2c_master_send(atmel_data->client, cPointer, 21))
                    goto i2c_err;

                // T27 Two-Touch Gesture
                cPointer[0] = cMM_Table[80];
                cPointer[1] = cMM_Table[81];
                cPointer[2] = 0x03;
                cPointer[3] = 0x02;
                cPointer[4] = 0x00;
                cPointer[5] = 0xE0;
                cPointer[6] = 0x03;
                cPointer[7] = 0x23;
                if (8 != i2c_master_send(atmel_data->client, cPointer, 8))
                    goto i2c_err;

                // T28 CTE Configuration
                cPointer[0] = cMM_Table[86];
                cPointer[1] = cMM_Table[87];
                cPointer[2] = 0x00;
                cPointer[3] = 0x00;
                cPointer[4] = 0x03;
                cPointer[5] = 0x04;
                cPointer[6] = 0x08;
                if (7 != i2c_master_send(atmel_data->client, cPointer, 7))
                    goto i2c_err;

            } else {
                // config for HW Version 1.5, 1.6, 1.0
                pr_info("[TS] config for HW Version 1.5, 1.6, 1.0\n");

                // check User Data Version
                // point to T38 User Data
                cPointer[0] = cMM_Table[20];
                cPointer[1] = cMM_Table[21];
                if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
                    goto i2c_err;

                if (2 != i2c_master_recv(atmel_data->client, cPointer, 2))
                    goto i2c_err;

                pr_info("[TS] User Data Version = 0x%02X\n", cPointer[1]);

                if ((cPointer[0]<0x10) || (cPointer[1]<USER_DATA_VERSION)) {
                    pr_info("[TS] need to back up\n");

                    // point to T38 User Data
                    cPointer[0] = cMM_Table[20];
                    cPointer[1] = cMM_Table[21];
                    cPointer[2] = 0x10;
                    cPointer[3] = USER_DATA_VERSION;
                    if (4 != i2c_master_send(atmel_data->client, cPointer, 4))
                        goto i2c_err;

                    // T7 Power Configuration
                    cPointer[0] = cMM_Table[26];
                    cPointer[1] = cMM_Table[27];
                    cPointer[2] = 0x20;
                    cPointer[3] = 0x0A;
                    cPointer[4] = 0x32;
                    if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                        goto i2c_err;

                    // T8 Acquire Configuration
                    cPointer[0] = cMM_Table[32];
                    cPointer[1] = cMM_Table[33];
                    cPointer[2] = 0x06;
                    cPointer[3] = 0x05;
                    cPointer[4] = 0x14;
                    cPointer[5] = 0x14;
                    cPointer[6] = 0x00;
                    cPointer[7] = 0x00;
                    cPointer[8] = 0x0A;
                    cPointer[9] = 0x37;
                    if (10 != i2c_master_send(atmel_data->client, cPointer, 10))
                        goto i2c_err;

                    if (cMM_Table[2] == 0x15) {
                        // config for HW Version 1.5
                        // T9 Multiple Touch
                        cPointer[0] = cMM_Table[38];
                        cPointer[1] = cMM_Table[39];
                        cPointer[2] = 0x83;
                        cPointer[3] = 0x00;
                        cPointer[4] = 0x00;
                        cPointer[5] = 0x10;
                        cPointer[6] = 0x0A;
                        cPointer[7] = 0x00;
                        cPointer[8] = 0x20;
                        cPointer[9] = 0x1E;
                        cPointer[10] = 0x02;
                        cPointer[11] = 0x03;
                        cPointer[12] = 0x00;
                        cPointer[13] = 0x00;
                        cPointer[14] = 0x00;
                        cPointer[15] = 0x0F;
                        cPointer[16] = 0x03;
                        cPointer[17] = 0x0A;
                        cPointer[18] = 0x0A;
                        cPointer[19] = 0x0A;
                        cPointer[20] = 0x20;
                        cPointer[21] = 0x03;
                        cPointer[22] = 0xE0;
                        cPointer[23] = 0x01;
                        cPointer[24] = 0x14;
                        cPointer[25] = 0x14;
                        cPointer[26] = 0x14;
                        cPointer[27] = 0x14;
                        cPointer[28] = 0x96;
                        cPointer[29] = 0x33;
                        cPointer[30] = 0x9A;
                        cPointer[31] = 0x52;
                        cPointer[32] = 0x00;
                        if (33 != i2c_master_send(atmel_data->client, cPointer, 33))
                            goto i2c_err;

                    } else {
                        // config for HW Version 1.6, 1.0
                        // T9 Multiple Touch
                        cPointer[0] = cMM_Table[38];
                        cPointer[1] = cMM_Table[39];
                        cPointer[2] = 0x83;
                        cPointer[3] = 0x00;
                        cPointer[4] = 0x00;
                        cPointer[5] = 0x10;
                        cPointer[6] = 0x0A;
                        cPointer[7] = 0x00;
                        cPointer[8] = 0x20;
                        cPointer[9] = 0x41;
                        cPointer[10] = 0x02;
                        cPointer[11] = 0x03;
                        cPointer[12] = 0x00;
                        cPointer[13] = 0x00;
                        cPointer[14] = 0x00;
                        cPointer[15] = 0x0F;
                        cPointer[16] = 0x03;
                        cPointer[17] = 0x1E;
                        cPointer[18] = 0x1E;
                        cPointer[19] = 0x0A;
                        cPointer[20] = 0x20;
                        cPointer[21] = 0x03;
                        cPointer[22] = 0xE0;
                        cPointer[23] = 0x01;
                        cPointer[24] = 0x14;
                        cPointer[25] = 0x14;
                        cPointer[26] = 0x14;
                        cPointer[27] = 0x14;
                        cPointer[28] = 0x96;
                        cPointer[29] = 0x33;
                        cPointer[30] = 0x9A;
                        cPointer[31] = 0x52;
                        cPointer[32] = 0x00;
                        if (33 != i2c_master_send(atmel_data->client, cPointer, 33))
                            goto i2c_err;
                    }

                    if (cMM_Table[2] == 0x15) {
                        // config for HW Version 1.5
                        // T15 Key Array
                        cPointer[0] = cMM_Table[44];
                        cPointer[1] = cMM_Table[45];
                        cPointer[2] = 0x83;
                        cPointer[3] = 0x10;
                        cPointer[4] = 0x0A;
                        cPointer[5] = 0x03;
                        cPointer[6] = 0x01;
                        cPointer[7] = 0x00;
                        cPointer[8] = 0x10;
                        cPointer[9] = 0x28;
                        cPointer[10] = 0x02;
                        if (11 != i2c_master_send(atmel_data->client, cPointer, 11))
                            goto i2c_err;

                    } else {
                        // config for HW Version 1.6, 1.0
                        // T15 Key Array
                        cPointer[0] = cMM_Table[44];
                        cPointer[1] = cMM_Table[45];
                        cPointer[2] = 0x83;
                        cPointer[3] = 0x10;
                        cPointer[4] = 0x0A;
                        cPointer[5] = 0x03;
                        cPointer[6] = 0x01;
                        cPointer[7] = 0x00;
                        cPointer[8] = 0x00;
                        cPointer[9] = 0x2D;
                        cPointer[10] = 0x02;
                        if (11 != i2c_master_send(atmel_data->client, cPointer, 11))
                            goto i2c_err;
                    }

                    // T20 Grip/Face Suppression
                    cPointer[0] = cMM_Table[62];
                    cPointer[1] = cMM_Table[63];
                    cPointer[2] = 0x00;
                    cPointer[3] = 0x64;
                    cPointer[4] = 0x64;
                    cPointer[5] = 0x64;
                    cPointer[6] = 0x64;
                    cPointer[7] = 0x03;
                    cPointer[8] = 0x00;
                    cPointer[9] = 0x1E;
                    cPointer[10] = 0x14;
                    cPointer[11] = 0x04;
                    cPointer[12] = 0x15;
                    cPointer[13] = 0x05;
                    if (14 != i2c_master_send(atmel_data->client, cPointer, 14))
                        goto i2c_err;

                    // T22 Noise Suppression
                    cPointer[0] = cMM_Table[68];
                    cPointer[1] = cMM_Table[69];
                    cPointer[2] = 0x0D;
                    cPointer[3] = 0x00;
                    cPointer[4] = 0x00;
                    cPointer[5] = 0x00;
                    cPointer[6] = 0x00;
                    cPointer[7] = 0x00;
                    cPointer[8] = 0x00;
                    cPointer[9] = 0x00;
                    cPointer[10] = 0x1E;
                    cPointer[11] = 0x00;
                    cPointer[12] = 0x01;
                    cPointer[13] = 0x00;
                    cPointer[14] = 0x0A;
                    cPointer[15] = 0x0F;
                    cPointer[16] = 0xFF;
                    cPointer[17] = 0xFF;
                    cPointer[18] = 0x00;
                    if (19 != i2c_master_send(atmel_data->client, cPointer, 19))
                        goto i2c_err;

                    // T24 One-Touch Gesture
                    cPointer[0] = cMM_Table[80];
                    cPointer[1] = cMM_Table[81];
                    cPointer[2] = 0x00;
                    cPointer[3] = 0x03;
                    cPointer[4] = 0xFF;
                    cPointer[5] = 0x03;
                    cPointer[6] = 0x00;
                    cPointer[7] = 0x64;
                    cPointer[8] = 0x64;
                    cPointer[9] = 0x01;
                    cPointer[10] = 0x0A;
                    cPointer[11] = 0x14;
                    cPointer[12] = 0x28;
                    cPointer[13] = 0x4B;
                    cPointer[14] = 0x00;
                    cPointer[15] = 0x02;
                    cPointer[16] = 0x00;
                    cPointer[17] = 0x64;
                    cPointer[18] = 0x00;
                    cPointer[19] = 0x19;
                    cPointer[20] = 0x00;
                    if (21 != i2c_master_send(atmel_data->client, cPointer, 21))
                        goto i2c_err;

                    // T25 Self Test
                    cPointer[0] = cMM_Table[86];
                    cPointer[1] = cMM_Table[87];
                    cPointer[2] = 0x00;
                    cPointer[3] = 0x00;
                    cPointer[4] = 0xEC;
                    cPointer[5] = 0x2C;
                    cPointer[6] = 0x4C;
                    cPointer[7] = 0x1D;
                    cPointer[8] = 0xEC;
                    cPointer[9] = 0x2C;
                    cPointer[10] = 0x4C;
                    cPointer[11] = 0x1D;
                    cPointer[12] = 0x00;
                    cPointer[13] = 0x00;
                    if (14 != i2c_master_send(atmel_data->client, cPointer, 14))
                        goto i2c_err;

                    // T27 Two-Touch Gesture
                    cPointer[0] = cMM_Table[92];
                    cPointer[1] = cMM_Table[93];
                    cPointer[2] = 0x00;
                    cPointer[3] = 0x02;
                    cPointer[4] = 0x00;
                    cPointer[5] = 0xE0;
                    cPointer[6] = 0x03;
                    cPointer[7] = 0x23;
                    if (8 != i2c_master_send(atmel_data->client, cPointer, 8))
                        goto i2c_err;

                    // T28 CTE Configuration
                    cPointer[0] = cMM_Table[98];
                    cPointer[1] = cMM_Table[99];
                    cPointer[2] = 0x00;
                    cPointer[3] = 0x00;
                    cPointer[4] = 0x03;
                    cPointer[5] = 0x20;
                    cPointer[6] = 0x18;
                    if (7 != i2c_master_send(atmel_data->client, cPointer, 7))
                        goto i2c_err;

                    // back up settings to the non-volatile memory
                    // point to BACKUPNV (Byte 1 of T6 Command Processor)
                    start_position = (((uint16_t)cMM_Table[15]<<8) | (uint16_t)cMM_Table[14]) + 1;
                    cPointer[0] = (uint8_t)(start_position & 0xFF);
                    cPointer[1] = (uint8_t)((start_position>>8) & 0xFF);
                    cPointer[2] = 0x55;
                    if (3 != i2c_master_send(atmel_data->client, cPointer, 3))
                        goto i2c_err;
                }
            }

            if (!gpio_get_value(108)) {
                // point to T5 Message Processor
                cPointer[0] = cMM_Table[8];
                cPointer[1] = cMM_Table[9];
                if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
                    goto i2c_err;

                if (9 != i2c_master_recv(atmel_data->client, cPointer, 9))
                    goto i2c_err;

                // REPORT ID of T6 Command Processor is 1
                // Bit 4 of Message Data for T6 is CAL
                if((cPointer[0]==1) && (cPointer[1]&0x10)) {
                    if (cMM_Table[2]==0x16 || cMM_Table[2]==0x10)
                        // touch/antitouch debug mode is only supported for verion 1.6, 1.0
                        check_chip_calibration();
                }
            }

            mdelay(100);

            break;


        case SUSPEND:
            if (cMM_Table[2] == 0x14) {
                // suspend for HW Version 1.4
                // point to T7 Power Configuration
                cPointer[0] = cMM_Table[20];
                cPointer[1] = cMM_Table[21];
                cPointer[2] = 0x00;
                cPointer[3] = 0x00;
                cPointer[4] = 0x00;
                if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                    goto i2c_err;

            } else {
                // suspend for HW Version 1.5, 1.6, 1.0
                // point to T7 Power Configuration
                cPointer[0] = cMM_Table[26];
                cPointer[1] = cMM_Table[27];
                cPointer[2] = 0x00;
                cPointer[3] = 0x00;
                cPointer[4] = 0x00;
                if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                    goto i2c_err;
            }

            break;


        case RESUME:
            if (cMM_Table[2] == 0x14) {
                // resume for HW Version 1.4
                // point to T7 Power Configuration
                cPointer[0] = cMM_Table[20];
                cPointer[1] = cMM_Table[21];
                cPointer[2] = 0x20;
                cPointer[3] = 0x0A;
                cPointer[4] = 0x32;
                if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                    goto i2c_err;

            } else {
                // resume for HW Version 1.5, 1.6, 1.0
                // point to T7 Power Configuration
                cPointer[0] = cMM_Table[26];
                cPointer[1] = cMM_Table[27];
                cPointer[2] = 0x20;
                cPointer[3] = 0x0A;
                cPointer[4] = 0x32;
                if (5 != i2c_master_send(atmel_data->client, cPointer, 5))
                    goto i2c_err;
            }

            check_chip_delta();

            break;

        default:
            break;
    }

    // point to T5 Message Processor
    cPointer[0] = cMM_Table[8];
    cPointer[1] = cMM_Table[9];
    if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
        goto i2c_err;

    return 0;

i2c_err:
    pr_err("[TS][%s] i2c error (%d)\n", __func__, status);
    return -ENXIO;
}
#endif


static void atmel_work_func(struct work_struct *work)
{
    int idx;
    uint8_t cPointer[2] = {0};
    uint8_t data[9] = {0};
    uint8_t kpd_data[2] = {0};

    static uint8_t cal_check_flag = 0;

    static touch_message Touch[TOUCH_NUMBER];

    while (!gpio_get_value(108)) {
        // point to T5 Message Processor
        cPointer[0] = cMM_Table[8];
        cPointer[1] = cMM_Table[9];
        if (2 != i2c_master_send(atmel_data->client, cPointer, 2))
            pr_err("[TS][%s] point to T5 error\n", __func__);

        if (9 != i2c_master_recv(atmel_data->client, data, 9))
            pr_err("[TS][%s] get message error\n", __func__);

        // Report ID of T6 Command Processor is 1
        if (data[0] == 1) {
            // CAL (Bit 4) of STATUS Field (Byte 1)
            if (data[1] & 0x10) {
                pr_info("[TS] calibration done\n");
                cal_check_flag = 1;
            }

        // Report IDs of T9 Multiple Touch are 2 ~ 11
        // we support TOUCH_NUMBER points at most of multiple touch
        } else if ((data[0]>=2) && (data[0]<=11) && (data[0]-2<TOUCH_NUMBER)) {
            if (cal_check_flag) {
                cal_check_flag = 0;
                if (cMM_Table[2]==0x16 || cMM_Table[2]==0x10)
                    // touch/antitouch debug mode is only supported for verion 1.6, 1.0
                    check_chip_calibration();
            }

            idx = data[0] - 2;

            // updata touch message
            // RELEASE (Bit 5) of STATUS Field (Byte 1)
            Touch[idx].pressed = !((data[1]>>5)&0x01);
            // XPOSMSB, YPOSMSB, XYPOSLSB (Byte 2, 3, 4)
            // X, Y are 10-bit Format
            Touch[idx].xPosition = ((uint16_t)data[2]<<2) | (((uint16_t)data[4]>>6)&0x0003);
            Touch[idx].yPosition = ((uint16_t)data[3]<<2) | (((uint16_t)data[4]>>2)&0x0003);
            // TCHAREA (Byte 5)
            Touch[idx].size = data[5];

            // report touch message
            for (idx=0; idx<TOUCH_NUMBER; idx++) {
                if (Touch[idx].pressed) {
                    input_report_abs(atmel_data->input, ABS_MT_POSITION_X, Touch[idx].xPosition);
                    input_report_abs(atmel_data->input, ABS_MT_POSITION_Y, Touch[idx].yPosition);
                    input_report_abs(atmel_data->input, ABS_MT_WIDTH_MAJOR, Touch[idx].size);
                }
                input_report_abs(atmel_data->input, ABS_MT_TOUCH_MAJOR, Touch[idx].pressed);
                input_mt_sync(atmel_data->input);
            }
            input_sync(atmel_data->input);

        // Report ID of T15 Key Array is 12
        } else if (data[0] == 12) {
            // KEYSTATE (Byte 2) of Message Data for T15 Key Array
            kpd_data[0] = data[2] & 0x07;
            a3_keypad_report_key(0, kpd_data);

        } else {
            mdelay(1);
            pr_debug("[TS][%s] data[0] = %d \n", __func__, data[0]);
        }
    }
}

static void atmel_calib_work_func(struct work_struct *work)
{
    set_mode(RESUME);
}

static irqreturn_t atmel_ts_interrupt(int irq, void *dev_id)
{
    disable_irq(irq);
    schedule_work(&atmel_data->work);
    enable_irq(atmel_data->client->irq);

    return IRQ_HANDLED;
}

static int __init atmel_register_input(struct input_dev *input)
{
    input->name = TS_DRIVER_NAME;
    input->id.bustype = BUS_I2C;
    input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    input->keybit[BIT_WORD(BTN_2)] = BIT_MASK(BTN_2);

    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
    input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, ATMEL_TS_X_MAX, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_X, ATMEL_TS_X_MIN, ATMEL_TS_X_MAX, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, ATMEL_TS_Y_MIN, ATMEL_TS_Y_MAX, 0, 0);

    return input_register_device(input);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_early_suspend(struct early_suspend *h)
{
    pr_debug("[TS] Enter %s\n", __func__);
    disable_irq(atmel_data->client->irq);
    set_mode(SUSPEND);
}

void atmel_late_resume(struct early_suspend *h)
{
    pr_info("[TS] Enter %s\n", __func__);
    schedule_work(&atmel_data->calib_work);
    enable_irq(atmel_data->client->irq);
}
#endif

static int atmel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    atmel_data = kzalloc(sizeof(struct atmel_data), GFP_KERNEL);
    if (atmel_data == NULL)
        return -ENOMEM;

    atmel_data->client = client;
    atmel_data->platform_data = (struct atmel_platform_data*)client->dev.platform_data;

    msleep(100);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENOTSUPP;

    strlcpy(client->name, TS_DRIVER_NAME, I2C_NAME_SIZE);
    i2c_set_clientdata(client, atmel_data);
    if (set_mode(INIT) == 0) {
        Check_Touch = 0;

        INIT_WORK(&atmel_data->work, atmel_work_func);
        INIT_WORK(&atmel_data->calib_work, atmel_calib_work_func);
        atmel_data->input = input_allocate_device();
        if (atmel_data->input == NULL)
            return -ENOMEM;

        if (atmel_register_input(atmel_data->input))
            goto set_mode_err;

        if (client->irq) {
            if (request_irq(client->irq, atmel_ts_interrupt, IRQF_TRIGGER_FALLING,
                            TS_DRIVER_NAME, atmel_data))
                goto request_irq_err;
        }
#if USEINIT
        disable_irq(atmel_data->client->irq);
        //set_mode(INIT);
        enable_irq(atmel_data->client->irq);
#endif

#if USE_FS
        if (device_create_file(&client->dev, &ts_attrs))
            pr_err("[TS][%s] device_create_file ts_attrs error\n", __func__);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
        atmel_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
        atmel_data->early_suspend.suspend = atmel_early_suspend;
        atmel_data->early_suspend.resume = atmel_late_resume;
        register_early_suspend(&atmel_data->early_suspend);
#endif

        pr_info("[TS] atmel probe done\n");
        return 0;

    } else {
        pr_info("[TS] atmel set_mode error, initialize cypress\n");
        Check_Touch = 1;
        kfree(atmel_data);

        return -ENOTSUPP;
    }

request_irq_err:
    free_irq(client->irq, atmel_data);

set_mode_err:
    input_free_device(atmel_data->input);
    kfree(atmel_data);
    pr_err("[TS][%s] atmel probe error\n", __func__);

    return -ENOTSUPP;
}

static int atmel_remove(struct i2c_client *client)
{
    struct atmel_data *tp = i2c_get_clientdata(client);

    input_unregister_device(tp->input);
    free_irq(client->irq, tp);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&tp->early_suspend);
#endif
    kfree(atmel_data);

    return 0;
}

static const struct i2c_device_id atmel_id[] = {
    { TS_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver atmel_driver = {
    .probe        = atmel_probe,
    .remove       = atmel_remove,
    .id_table     = atmel_id,
    .driver       = {
        .name = TS_DRIVER_NAME,
    },
};

static int __init atmel_init(void)
{
    pr_debug("[TS] Enter %s\n", __func__);
    return i2c_add_driver(&atmel_driver);
}

static void __exit atmel_exit(void)
{
    i2c_del_driver(&atmel_driver);
}

module_init(atmel_init);
module_exit(atmel_exit);

MODULE_AUTHOR("Fanso Chen <Fanso_Chen@acer.com.tw>; "
              "Vincent Chen <Vincent_CH_Chen@acer.com.tw>");
MODULE_DESCRIPTION("ATMEL driver");
MODULE_LICENSE("GPL v2");

