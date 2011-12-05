#include <linux/i2c.h>
#include <mach/mcu.h>
#include "adi752x.h"
#include <linux/input.h>
#include <linux/delay.h>

struct CecData {
    uint8_t LA;
    uint8_t PA[2];
    uint8_t Header_directly;
    uint8_t Header_broadcast;
};

static struct i2c_client *p_main;
static struct i2c_client *p_cec;
static struct CecData cecdata;

void analyse_msg(uint8_t *msg)
{
    uint8_t key_buf[2] = {0x0,0x0};
    uint8_t Tx_frame[16] = {0x0};

    pr_info("[HDMI] cec_msg[0] = 0x%x cec_msg[1] = 0x%x  cec_msg[2] = 0x%x\n",
                     msg[0],msg[1],msg[2]);
    switch (msg[1]) {
        case CEC_OPCODE_USER_CONTROL_PRESSED:
            switch (msg[2]) {
                case CEC_UICMD_SELECT:
                    key_buf[0] = KEY_ENTER;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_UP:
                    key_buf[0] = KEY_UP;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_DOWN:
                    key_buf[0] = KEY_DOWN;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_LEFT:
                    key_buf[0] = KEY_LEFT;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_RIGHT:
                    key_buf[0] = KEY_RIGHT;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_MENU:
                    key_buf[0] = KEY_MENU;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_BACK:
                    key_buf[0] = KEY_BACK;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_PLAY:
                    key_buf[0] = KEY_PLAYPAUSE;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_STOP:
                    turn_off_HDMI();
                    break;
                case CEC_UICMD_PAUSE:
                    key_buf[0] = KEY_PLAYPAUSE;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_REWIND:
                    key_buf[0] = KEY_REWIND;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_FAST_FORWARD:
                    key_buf[0] = KEY_FASTFORWARD;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_FORWARD:
                    key_buf[0] = KEY_NEXTSONG;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case CEC_UICMD_BACKWARD:
                    key_buf[0] = KEY_PREVIOUSSONG;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case 0x40:
                    Tx_frame[0] = CEC_TX_FRAME_HEADER;
                    Tx_frame[1] = cecdata.Header_directly;
                    Tx_frame[2] = 0x90;
                    Tx_frame[3] = 0x00;
                    i2c_write(p_cec,Tx_frame,4);
                    i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x3);
                    i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
                    break;
            default:
                    break;
            }
            break;

        case CEC_OPCODE_PLAY:
            switch (msg[2]) {
                case 0x24:
                case 0x25:
                    key_buf[0] = KEY_PLAYPAUSE;
                    a3_keypad_report_key(2,key_buf);
                    key_buf[0] = 0x0;
                    a3_keypad_report_key(2,key_buf);
                    break;
            default:
                    break;
            }
            break;

        case CEC_OPCODE_DECK_CONTROL:
            switch (msg[2]) {
                case 0x1:
                    key_buf[0] = KEY_FASTFORWARD;
                    a3_keypad_report_key(2,key_buf);
                    key_buf[0] = 0x0;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case 0x2:
                    key_buf[0] = KEY_REWIND;
                    a3_keypad_report_key(2,key_buf);
                    key_buf[0] = 0x0;
                    a3_keypad_report_key(2,key_buf);
                    break;
                case 0x3: /* stop */
                    key_buf[0] = KEY_PLAYPAUSE;
                    a3_keypad_report_key(2,key_buf);
                    key_buf[0] = 0x0;
                    a3_keypad_report_key(2,key_buf);
                    break;
            default:
                    break;
            }
            break;

        case CEC_OPCODE_USER_CONTROL_RELEASED:
            key_buf[0] = 0x0;
            a3_keypad_report_key(2,key_buf);
            break;

        case CEC_OPCODE_GIVE_OSD_NAME:
            Tx_frame[0x0] = CEC_TX_FRAME_HEADER;
            Tx_frame[0x1] = cecdata.Header_directly;
            Tx_frame[0x2] = CEC_OPCODE_SET_OSD_NAME;
            Tx_frame[0x3] = 'A';
            Tx_frame[0x4] = 'c';
            Tx_frame[0x5] = 'e';
            Tx_frame[0x6] = 'r';
            Tx_frame[0x7] = '_';
            Tx_frame[0x8] = 'S';
            Tx_frame[0x9] = 't';
            Tx_frame[0xA] = 'r';
            Tx_frame[0xB] = 'e';
            Tx_frame[0xC] = 'a';
            Tx_frame[0xD] = 'm';

            i2c_write(p_cec,Tx_frame,14);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0xd);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_GIVE_DRVICE_VENDOR_ID:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_broadcast;
            Tx_frame[2] = 0x87;
            Tx_frame[3] = 0x00;
            Tx_frame[4] = 0x00;
            Tx_frame[5] = 0xE2;

            i2c_write(p_cec,Tx_frame,6);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x5);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_GIVE_DRVICE_POWER_STATUS:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_directly;
            Tx_frame[2] = 0x90;
            Tx_frame[3] = 0x00;

            i2c_write(p_cec,Tx_frame,4);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x3);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_CEC_VERSION:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_directly;
            Tx_frame[2] = 0x9E;
            Tx_frame[3] = 0x04;

            i2c_write(p_cec,Tx_frame,4);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x3);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_SET_STREAM_PATH:
         case CEC_OPCODE_REQUEST_ACTIVE_SOURCE:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_broadcast;
            Tx_frame[2] = 0x82;
            Tx_frame[3] = cecdata.PA[0];
            Tx_frame[4] = cecdata.PA[1];
            pr_info("[HDMI] cecdata.PA[0] = 0x%x  cecdata.PA[1] = 0x%x\n",cecdata.PA[0],cecdata.PA[1]);
            i2c_write(p_cec,Tx_frame,5);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x4);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_GIVE_PHYSICAL_ADDRESS:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_broadcast;
            Tx_frame[2] = 0x84;
            Tx_frame[3] = cecdata.PA[0];
            Tx_frame[4] = cecdata.PA[1];
            Tx_frame[5] = 0x4;

            i2c_write(p_cec,Tx_frame,6);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x5);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_GIVE_DECK_STATUS:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_directly;
            Tx_frame[2] = 0x1B;
            Tx_frame[3] = 0x01;

            i2c_write(p_cec,Tx_frame,4);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x3);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

         case CEC_OPCODE_MENU_REQUEST:
            Tx_frame[0] = CEC_TX_FRAME_HEADER;
            Tx_frame[1] = cecdata.Header_directly;
            Tx_frame[2] = 0x8E;
            Tx_frame[3] = 0x01;
            i2c_write(p_cec,Tx_frame,4);
            i2c_write_byte(p_cec,CEC_TX_FRAME_LENGTH,0x3);
            i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
            break;

        default:
            break;
    }
    i2c_write_byte(p_cec,CEC_RX_ENABLE,0x1);
}

int cec_find_la(void)
{
    /*  Main interrupt interrupt mask */
    uint8_t buf = 0;
    uint8_t LA[3] = {CEC_LA_CANDIDATE};
    uint8_t i =0;

    for(i = 0 ; i < 3 ; i++) {
        i2c_write_byte(p_cec,CEC_TX_FRAME_HEADER,(LA[i] << 4 | LA[i]));
        i2c_write_byte(p_cec,CEC_TX_ENABLE,0x1);
        msleep(300);
        i2c_read( p_cec,CEC_TX_NACK_COUNTER,&buf,1);
        pr_info("[HDMI] Logical(%d) = %d\n",LA[i],buf);
        if(( buf & 0xf) != 0)
            return LA[i];
    }

    return 0xf;
}

int cec_Rx_Setting(void)
{
    /*  Main interrupt interrupt mask */
    uint8_t cec_int[2] = {0x95,0xff};

    if(i2c_read(p_main,cec_int[0], &cec_int[0], 1))
        goto cec_Rx_Setting_err;

    cec_int[1] = cec_int[0]|0x04;
    cec_int[0] = 0x95;
    if(i2c_write( p_main, cec_int, 2))
        goto cec_Rx_Setting_err;
    if(i2c_write_byte(p_cec,CEC_RX_LOGICAL_ADDRESS0,(0xf << 4 | cecdata.LA)))
        goto cec_Rx_Setting_err;
    if(i2c_write_byte(p_cec,CEC_RX_LOGICAL_ADDRESS_MASK,0x10))
        goto cec_Rx_Setting_err;
    if(i2c_write_byte(p_cec,CEC_RX_ENABLE,0x1))
        goto cec_Rx_Setting_err;

    return 0;

cec_Rx_Setting_err:
    pr_err("[HDMI]%s error \n",__func__);
    return -1;
}

void init_cec(struct i2c_client *main,struct i2c_client *cec ,struct edid_data ed)
{
    uint8_t u8_st_total_hi[] = {0x2d,CEC_INITIALIZE_2D_48};
    uint8_t u8_rise_time_hi[] = {0x4a,CEC_INITIALIZE_4A_4B};
    uint8_t u8_bit_low_one_min_hi[] = {0x4d,CEC_INITIALIZE_4D_52};

    p_main = main;
    p_cec = cec;

    /*  cec power on*/
    if(i2c_write_byte(p_cec, CEC_CLOCK_DIVIDER_POWER_MODE, 0x65))
        goto init_cec_err;

    cecdata.LA = cec_find_la();
    cecdata.Header_directly = cecdata.LA << 4 | 0x0;
    cecdata.Header_broadcast = cecdata.LA << 4 | 0xf;
    cecdata.PA[0] = ed.hdmi_addr_ab;
    cecdata.PA[1] = ed.hdmi_addr_cd;
    if(i2c_write(p_cec,u8_st_total_hi,sizeof(u8_st_total_hi)))
        goto init_cec_err;
    if(i2c_write(p_cec,u8_rise_time_hi,sizeof(u8_rise_time_hi)))
        goto init_cec_err;
    if(i2c_write(p_cec,u8_bit_low_one_min_hi,sizeof(u8_bit_low_one_min_hi)))
        goto init_cec_err;

    /*  cec_int */
    if(cec_Rx_Setting())
        goto init_cec_err;
    return;

init_cec_err:
    return;
}




