/* First release: 2009.08.04
 * ADI ADV752X HDMI IC
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/screen_info.h>
#include <../arch/arm/mach-msm/clock.h>
#include <../arch/arm/mach-msm/proc_comm.h>
#include "msm/lcdc_samsung.h"
#include "msm/mdp.h"
#include "msm/msm_fb.h"
#include <mach/vreg.h>
#include <mach/mcu.h>
#include <linux/clk.h>
#include <linux/switch.h>
#include <linux/earlysuspend.h>
#include <linux/pm_qos_params.h>

#include "adi752x.h"

#define ADI752x_DRIVER_NAME  "adi752x"

uint16_t os_rsolution[] = {
    640,480,        // HTX_480i
    720,576,        // HTX_576i
    720,480,        // HTX_720_480p
    720,576,        // HTX_720_576p
    1280,720,       // HTX_720p_60
    1280,720,       // HTX_720p_50
    1920,1080,      // HTX_1080i_30
    1920,1080,      // HTX_1080i_25
    640,480,        // HTX_640_480p
    480,800         // Device LCD Resolution
};

extern int samsung_panel_on(void);
extern int samsung_panel_off(void);

/*
 *    Declaire bodies
 */
static const struct file_operations adi752x_fops = {
    .owner        = THIS_MODULE,
    .open         = adi752x_open,
    .release      = adi752x_close,
    .ioctl        = adi752x_ioctl,
};

struct adi752x_data {
    struct i2c_client *main;
    struct i2c_client *main_72;
    struct i2c_client *main_7B;
    struct i2c_client *cec;
    struct i2c_client *edid;
    struct work_struct work;
    struct work_struct edid_work;
    struct work_struct detach;
    bool flag_edid_ready;
    uint8_t hdmi_status;
    uint8_t intent_status;
    int ioctl_mode;
    int tv_format_backup;
    struct timer_list edid_timer;
    struct timer_list detach_timer;

    /* 8:HTX_640_480p; 5:HTX_720p_50; 4: HTX_720p_60; 3:HTX_720_576p; 2:HTX_720_480p*/
    int set_tv_format;
    int locked_format;

    struct edid_data edid_info;
#if USE_AV_MUTE
    struct work_struct av_mute;
    struct timer_list av_mute_timer;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

static struct miscdevice adi752x_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = ADI752x_DRIVER_NAME,
    .fops = &adi752x_fops,
};

static struct adi752x_data *p_adi752x;

enum {
    NO_PLUGIN           = 0,
    HDMI_PLUGIN,
    HDMI_ACTIVE,
    HDMI_UNACTIVE,
    HDMI_IC_ON,
    HDMI_IC_OFF,
    INIT,
    RESUME,
    SUSPEND,
};

struct hdmi_res {
    struct switch_dev sdev;
};

static struct hdmi_res *hdp;
struct platform_device *pdev;

void set_hdmi_platform_device(struct platform_device *platform)
{
    pdev = platform;
}

void clean_edid_info(void)
{
    p_adi752x->edid_info.support_format = 0;
    p_adi752x->edid_info.vsdb_hdmi = 0;
    p_adi752x->edid_info.hdmi_addr_ab = 0;
    p_adi752x->edid_info.hdmi_addr_cd = 0;
    p_adi752x->edid_info.segment_num = 0;
    p_adi752x->edid_info.segment_current= 0;
}

/*
 *    Code bodies
 */
static ssize_t acer_hdmi_print_name(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "HDMI Status \n");
}

static ssize_t acer_hdmi_print_state(struct switch_dev *sdev, char *buf)
{
    p_adi752x->intent_status = switch_get_state(&hdp->sdev);
    switch (p_adi752x->intent_status) {
        case NO_PLUGIN:
            clean_edid_info();
            sprintf(buf, "0");
            break;
        case HDMI_PLUGIN:
            sprintf(buf, "1");
            break;
        case HDMI_ACTIVE:
            sprintf(buf, "2");
            break;
    }

    return -EINVAL;
}

void change_lcd_pll(uint8_t ImageSize)
{
    int lcdc_width;
    int lcdc_height;
    int lcdc_bpp;
    int lcdc_border_clr;
    int lcdc_underflow_clr;
    int lcdc_hsync_skew;

    int hsync_period;
    int hsync_ctrl;
    int vsync_period;
    int display_hctl;
    int display_v_start;
    int display_v_end;
    int active_hctl;
    int active_v_start;
    int active_v_end;
    int ctrl_polarity = 0;
    int h_back_porch;
    int h_front_porch;
    int v_back_porch;
    int v_front_porch;
    int hsync_pulse_width;
    int vsync_pulse_width;
    int hsync_polarity;
    int vsync_polarity;
    int data_en_polarity;
    int hsync_start_x;
    int hsync_end_x;
    unsigned long pm_qos_freq;
    struct fb_info *fbi;
    struct fb_var_screeninfo *var;
    struct fb_fix_screeninfo *fix;
    struct msm_fb_data_type *mfd;
    int bpp;
    uint8 *buf;


    struct msm_panel_info    vinfo={0};

    int nCLK = 0;
    int rc = 0;
    static unsigned int MyPCLK[] = {
        9600000,
        4800000,
        25175000,
        27027000,
        74250000,
    };

    unsigned id = MDP_LCDC_PCLK_CLK;

    unsigned rpc_id = 10;
    boolean T;

    switch (ImageSize) {
        case HTX_640_480p:
            pr_debug("[HDMI] HTX_640_480p\n");

            /* change pixel clock */
            nCLK = 2;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = FALSE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 640;
            vinfo.yres = 480;
            vinfo.clk_rate = 25175000;    /* 25.175 Mhz */

            vinfo.lcdc.h_back_porch = 48;
            vinfo.lcdc.h_front_porch = 16;
            vinfo.lcdc.h_pulse_width = 96;
            vinfo.lcdc.v_back_porch = 33;
            vinfo.lcdc.v_front_porch = 10;
            vinfo.lcdc.v_pulse_width = 2;

            hsync_polarity = 1;
            vsync_polarity = 1;
            data_en_polarity = 0;

            break;

        case HTX_720_480p:
            pr_debug("[HDMI] HTX_720_480p\n");

            nCLK = 3;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = FALSE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 720;
            vinfo.yres = 480;
            vinfo.clk_rate = 27027000;

            vinfo.lcdc.h_back_porch = 60;
            vinfo.lcdc.h_front_porch = 16;
            vinfo.lcdc.h_pulse_width = 62;
            vinfo.lcdc.v_back_porch = 30;
            vinfo.lcdc.v_front_porch = 9;
            vinfo.lcdc.v_pulse_width = 6;

            hsync_polarity = 1;
            vsync_polarity = 1;
            data_en_polarity = 0;

            break;

        case HTX_720_576p:
            pr_debug("[HDMI] HTX_720_576p\n");

            nCLK = 3;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = FALSE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 720;
            vinfo.yres = 576;
            vinfo.clk_rate = 27027000;

            vinfo.lcdc.h_back_porch = 68;
            vinfo.lcdc.h_front_porch = 12;
            vinfo.lcdc.h_pulse_width = 64;
            vinfo.lcdc.v_back_porch = 39;
            vinfo.lcdc.v_front_porch = 5;
            vinfo.lcdc.v_pulse_width = 5;

            hsync_polarity = 1;
            vsync_polarity = 1;
            data_en_polarity = 0;

            break;


        case HTX_720p_60:
            pr_debug("[HDMI] HTX_720p_60\n");

            nCLK = 4;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = FALSE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 1280;
            vinfo.yres = 720;
            vinfo.clk_rate = 74250000;

            vinfo.lcdc.h_back_porch = 220;
            vinfo.lcdc.h_front_porch = 110;
            vinfo.lcdc.h_pulse_width = 40;
            vinfo.lcdc.v_back_porch = 20;
            vinfo.lcdc.v_front_porch = 5;
            vinfo.lcdc.v_pulse_width = 5;

            hsync_polarity = 0;
            vsync_polarity = 0;
            data_en_polarity = 0;

            break;

        case HTX_720p_50:
            pr_debug("[HDMI] HTX_720p_50\n");

            nCLK = 4;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = FALSE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 1280;
            vinfo.yres = 720;
            vinfo.clk_rate = 74250000;

            vinfo.lcdc.h_back_porch = 220;
            vinfo.lcdc.h_front_porch = 440;
            vinfo.lcdc.h_pulse_width = 40;
            vinfo.lcdc.v_back_porch = 20;
            vinfo.lcdc.v_front_porch = 5;
            vinfo.lcdc.v_pulse_width = 5;

            hsync_polarity = 0;
            vsync_polarity = 0;
            data_en_polarity = 0;

            break;

        case LCD_480_800:
            pr_debug("[HDMI] LCD_480_800\n");

            nCLK = 2;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            pr_debug("[HDMI] PCLK = %d, rc = %d\n", MyPCLK[nCLK], rc);

            /* Invert PCLK waveform*/
            T = TRUE;
            rc = msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_CLK_INV, &rpc_id, &T);

            ssleep(1);

            vinfo.xres = 480;
            vinfo.yres = 800;
            vinfo.clk_rate = 24576000;

            vinfo.lcdc.h_back_porch = 8;
            vinfo.lcdc.h_front_porch = 8;
            vinfo.lcdc.h_pulse_width = 1;
            vinfo.lcdc.v_back_porch = 4;
            vinfo.lcdc.v_front_porch = 4;
            vinfo.lcdc.v_pulse_width = 4;

            hsync_polarity = 1;
            vsync_polarity = 1;
            data_en_polarity = 1;

            break;

        default:
            pr_debug("[HDMI] default: 640_480p\n");

            nCLK = 2;
            rc = msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &MyPCLK[nCLK]);
            ssleep(1);

            vinfo.xres = 640;
            vinfo.yres = 480;
            vinfo.clk_rate = 25175000;

            vinfo.lcdc.h_back_porch = 48;
            vinfo.lcdc.h_front_porch = 16;
            vinfo.lcdc.h_pulse_width = 96;
            vinfo.lcdc.v_back_porch = 33;
            vinfo.lcdc.v_front_porch = 10;
            vinfo.lcdc.v_pulse_width = 2;

            hsync_polarity = 1;
            vsync_polarity = 1;
            data_en_polarity = 0;

            break;
    }

    if (vinfo.clk_rate > 62000000)
        pm_qos_freq = 128000;    /* pm_qos_freq should be in Khz */
    else
        pm_qos_freq = 62000;
    pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "lcdc",
                                      pm_qos_freq);

    mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);
    fbi = mfd->fbi;
    var = &fbi->var;
    fix = &fbi->fix;

    bpp = fbi->var.bits_per_pixel / 8;
    buf = (uint8 *) fbi->fix.smem_start;

    if (fix->line_length != FB_XRES * bpp) {
        fix->line_length = FB_XRES * bpp;
    }
    buf +=
        (fbi->var.xoffset + fbi->var.yoffset * FB_XRES) * bpp;

    vinfo.type = LCDC_PANEL;
    vinfo.pdest = DISPLAY_1;
    vinfo.wait_cycle = 0;
    vinfo.bpp =VINFO_BPP;
    vinfo.fb_num = 2;

    MDP_OUTP(MDP_BASE + 0x90008, (uint32) buf);
    /* active window width and height */
    MDP_OUTP(MDP_BASE + 0x90004, ((vinfo.yres) << 16) | (vinfo.xres));


    if(vinfo.xres != 720)
        MDP_OUTP(MDP_BASE + 0x9000c, vinfo.xres * (vinfo.bpp/8));
    else
        MDP_OUTP(MDP_BASE + 0x9000c, FB_XRES * (vinfo.bpp/8));

    /* x/y coordinate = always 0 for lcdc */
    MDP_OUTP(MDP_BASE + 0x90010, 0);

    /*
     * LCDC timing setting
     */
    h_back_porch = vinfo.lcdc.h_back_porch;
    h_front_porch = vinfo.lcdc.h_front_porch;
    v_back_porch = vinfo.lcdc.v_back_porch;
    v_front_porch = vinfo.lcdc.v_front_porch;
    hsync_pulse_width = vinfo.lcdc.h_pulse_width;
    vsync_pulse_width = vinfo.lcdc.v_pulse_width;

    lcdc_border_clr = 0;
    lcdc_underflow_clr = 0x00;
    lcdc_hsync_skew = 0;

    lcdc_width = vinfo.xres;
    lcdc_height = vinfo.yres;
    lcdc_bpp = vinfo.bpp;

    hsync_period =
        hsync_pulse_width + h_back_porch + lcdc_width + h_front_porch;
    hsync_ctrl = (hsync_period << 16) | hsync_pulse_width;
    hsync_start_x = hsync_pulse_width + h_back_porch;
    hsync_end_x = hsync_period - h_front_porch - 1;
    display_hctl = (hsync_end_x << 16) | hsync_start_x;

    vsync_period =
        (vsync_pulse_width + v_back_porch + lcdc_height +
         v_front_porch) * hsync_period;
    display_v_start =
        (vsync_pulse_width + v_back_porch) * hsync_period + lcdc_hsync_skew;
    display_v_end =
        vsync_period - (v_front_porch * hsync_period) + lcdc_hsync_skew - 1;

    active_hctl = 0;
    active_v_start = 0;
    active_v_end = 0;

    ctrl_polarity = (data_en_polarity << 2) |(vsync_polarity << 1) | (hsync_polarity);

    MDP_OUTP(MDP_BASE + LCDC_BASE , 0);

    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x4, hsync_ctrl);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x8, vsync_period);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0xc, vsync_pulse_width * hsync_period);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x10, display_hctl);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x14, display_v_start);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x18, display_v_end);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x28, lcdc_border_clr);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x2c, lcdc_underflow_clr);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x30, lcdc_hsync_skew);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x38, ctrl_polarity);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x1c, active_hctl);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x20, active_v_start);
    MDP_OUTP(MDP_BASE + LCDC_BASE + 0x24, active_v_end);

    MDP_OUTP(MDP_BASE + LCDC_BASE , 1);


}

int16_t get_bit( int16_t in, int16_t high_bit, int16_t low_bit )
{
    int16_t i;
    int16_t mask = 0;

    for( i = 0; i <= ( high_bit - low_bit ); i++ )
        mask += ( 1 << i );

        mask <<= low_bit;
        in &= mask;

    return (int16_t)( in >>= low_bit );
}


int i2c_read(struct i2c_client *client, uint8_t addr , uint8_t *buf , int length)
{
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg[2];

    if(!gpio_get_value(82)){
        pr_err("[HDMI] (%s) error GPIO 82 = 0 reg = 0x%x\n",__func__,addr);
        return -1;
    }
    msg[0].addr = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len = 1;
    msg[0].buf = &addr;
    msg[1].addr = client->addr;
    msg[1].flags = client->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len = length;
    msg[1].buf = buf;

    if(i2c_transfer(adap, msg, 2) < 0) {
        pr_err("[HDMI] (%s) error \n",__func__);
        return -1;
    }
    return 0;
}


int i2c_write(struct i2c_client *client, char *buf, int count)
{

    if(!gpio_get_value(82)){
        pr_err("[HDMI] (%s) error GPIO 82 = 0 reg = 0x%x\n",__func__,buf[0]);
        return -1;
    }

    if(count != i2c_master_send(client, buf, count)) {
        pr_err("[HDMI] (%s) error \n",__func__);
        return -1;
    }
    return 0;
}

int i2c_write_byte(struct i2c_client *client, uint8_t addr, uint8_t buf)
{
    uint8_t data[2] = {addr,buf};

    if(!gpio_get_value(82)){
        pr_err("[HDMI] (%s) error GPIO 82 = 0 reg = 0x%x\n",__func__,addr);
        return -1;
    }

    if(2 != i2c_master_send(client, data, 2)) {
        pr_err("[HDMI] (%s) error \n",__func__);
        return -1;
    }
    return 0;
}

int i2c_write_bit(struct i2c_client *client, uint8_t addr,uint8_t bit, bool en)
{
    uint8_t buf = 0;

    if(!gpio_get_value(82)){
        pr_err("[HDMI] (%s) error GPIO 82 = 0 reg = 0x%x\n",__func__,addr);
        return -1;
    }

    if(i2c_read(client,addr,&buf,1)) {
        pr_err("[HDMI] (%s) error \n",__func__);
        return -1;
    }

    buf = ((buf & (~(1 << bit))) | (en << bit));

    if(i2c_write_byte(client,addr,buf)) {
        pr_err("[HDMI] (%s) error \n",__func__);
        return -1;
    }
    return 0;
}


void fixd_registers( void )
{
    uint8_t buf = 0;
    int ret = 0;

    ret |= i2c_write_byte(p_adi752x->main,0x98,0x03);
    ret |= i2c_write_byte(p_adi752x->main,0x9C,0x38);

    ret |= i2c_read(p_adi752x->main,0x9D, &buf,1);
    buf = (buf & 0xFC) | 0x1;
    ret |= i2c_write_byte(p_adi752x->main,0x9D,buf);

    ret |= i2c_read(p_adi752x->main,0xA2, &buf,1);
    buf = (buf & 0x3) | 0x94;
    ret |= i2c_write_byte(p_adi752x->main,0xA2,buf);

    ret |= i2c_read(p_adi752x->main,0xA3, &buf,1);
    buf = (buf & 0x3) | 0x94;
    ret |= i2c_write_byte(p_adi752x->main,0xA3,0x94);

    ret |= i2c_write_byte(p_adi752x->main,0xDE,0x88);

    /* Underscanned */
    ret |= i2c_write_byte(p_adi752x->main,0x55,0x2);

    if(ret)
        pr_err("[HDMI] %s err \n",__func__);

}

void set_audio(void)
{
    uint8_t n[4];

    /* Reset CTS value*/
    if(i2c_write_byte(p_adi752x->main,0xA,0x81))
        goto set_audio_err;
    if(i2c_write_byte(p_adi752x->main,0xA,0x1))
        goto set_audio_err;

    /* HDMI or DVI*/
    if( p_adi752x->edid_info.vsdb_hdmi ) {
        i2c_write_bit(p_adi752x->main,0xAF, 1, 1);
    } else {
        i2c_write_bit(p_adi752x->main,0xAF, 1, 0);
    }

    /* one I2S channel */
    if(i2c_write_byte(p_adi752x->main,0x0C,0x84))
        goto set_audio_err;

    /* Sampling Freqency */
    if(i2c_write_byte(p_adi752x->main,0x15,0x20))
        goto set_audio_err;

    /* N value : 4096, 6272, 6144, 12544, 12288, 25088, 24576 */
    n[0] = 0x01;
    n[1] = (uint8_t)( ( 6272 >> 16 ) & 0x0F );
    n[2] = (uint8_t)( ( 6272 >> 8 ) & 0xFF );
    n[3] = (uint8_t)( 6272 & 0xFF );
    i2c_write( p_adi752x->main, n, 4);

    return;
set_audio_err:
    pr_err("[HDMI] %s error \n",__func__);

}


void set_video_mode(AspectRatio aspect_ratio )
{
    int ret = 0;

    /* Underscanned */
    ret |= i2c_write_byte(p_adi752x->main,0x55,0x2);

    if( aspect_ratio == _4x3 ) {
        /*  4x3 and active formate as same as picture aspect ratio.*/
        ret |= i2c_write_bit(p_adi752x->main,0x17,1,0);
        ret |= i2c_write_byte(p_adi752x->main,0x56,0x18);
    }
    if( aspect_ratio == _16x9 ) {
        /*  16x9 and active formate as same as picture aspect ratio.*/
        ret |= i2c_write_bit(p_adi752x->main,0x17,1,1);
        ret |= i2c_write_byte(p_adi752x->main,0x56,0x28);
    }

    if(ret)
        pr_err("[HDMI] %s err \n",__func__);
}


/* This function is passed to ADV752X_interrupt_handler(). It is called when EDID is */
/* finished processing. */
int setup_audio_video( void )
{
    int16_t supported_format;
    int16_t count_format = 0, set_format = 0;

    /* supported_video_format(); */
    supported_format = p_adi752x->edid_info.support_format;

    for ( count_format = p_adi752x->set_tv_format; count_format >= HTX_480i; count_format--) {

        set_format = supported_format & ( 1 << count_format );

        if((set_format|| (count_format < HTX_720_480p)|| (p_adi752x->ioctl_mode & 0x1))
            && (( HTX_1080i_25 != count_format)&& ( HTX_1080i_30 !=count_format ))) {

            if (count_format < HTX_720_480p) {
                count_format = HTX_640_480p;
                pr_err("[HDMI](%s) not have any formate\n",__func__);
            }

            change_lcd_pll(count_format);

            if (( HTX_720p_50 == count_format ) || ( HTX_720p_60 == count_format ))
                set_video_mode(_16x9);
            else
                set_video_mode(_4x3);

            set_audio();

            pr_info("[HDMI] locked format = %d  x %d  (0x%x)\n",
                        os_rsolution[count_format*2],os_rsolution[count_format*2+1],
                        supported_format);
            return count_format;

        }
    }

    pr_info( "[HDMI] %s, Finally, set_format to 480*800 \n", __func__);
    return LCD_480_800;
}

void i2s_enable_clock(void)
{
    /* enable I2S clock */
    static struct clk *sdac_clk;
    sdac_clk = clk_get(0, "sdac_clk");
    clk_enable(sdac_clk);
}

void i2s_disable_clock(void)
{
    /* disable I2S clock */
    static struct clk *sdac_clk;
    sdac_clk = clk_get(0, "sdac_clk");
    clk_disable(sdac_clk);
}

void turn_off_lcd( void )
{
    /* Turn off LCD panel */
    samsung_panel_off();
}

void turn_on_lcd( void )
{
    change_lcd_pll(LCD_480_800);

    /* Turn on LCD panel */
    samsung_panel_on();

    msleep(5);
}


int gpio_icPower_setting(uint8_t act)
{
    uint8_t tmp = 0;
    switch(act) {
        case HDMI_IC_ON:
            /* Turn on ic hw power */
            gpio_set_value(101, 1); /* set VOUT_5V */
            /* Turn off MOS */
            gpio_set_value(82, 1);
            if(i2c_write_bit(p_adi752x->main,0x41,6,0))
                goto setting_err;
            /* Change Package map address */
            i2c_write_byte(p_adi752x->main,0x45,0x66);
            break;

        case HDMI_IC_OFF:
            if(i2c_write_bit(p_adi752x->main,0x41,6,1))
                goto setting_err;
            gpio_set_value(82, 0);
            gpio_set_value(101, 1);
            break;

        case HDMI_ACTIVE:
            gpio_set_value(101, 1);
            gpio_set_value(82, 1);
            if(i2c_write_bit(p_adi752x->main,0x41,6,0))
                goto setting_err;
            i2c_write_byte(p_adi752x->main,0x45,0x66);
            for ( tmp = 111; tmp <= 138; tmp++) {
                if(tmp == 135)
                   gpio_tlmm_config(GPIO_CFG(tmp, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), GPIO_ENABLE);
                else
                   gpio_tlmm_config(GPIO_CFG(tmp, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), GPIO_ENABLE);
            }
            break;

        case HDMI_UNACTIVE:
            if(i2c_write_bit(p_adi752x->main,0x41,6,1))
                goto setting_err;
            gpio_set_value(82, 0);
            gpio_set_value(101, 1);
            for ( tmp = 111; tmp <= 138; tmp++) {
                gpio_tlmm_config(GPIO_CFG(tmp, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
            }
            break;

        case INIT:
            gpio_tlmm_config(GPIO_CFG(100, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
            gpio_tlmm_config(GPIO_CFG(82, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
            break;

        case SUSPEND:
            if(gpio_get_value(82)) {
                if(i2c_write_bit(p_adi752x->main,0x41,6,1)) {
                    goto setting_err;
                }
            }
            clean_edid_info();
            gpio_set_value(82, 0);
            gpio_tlmm_config(GPIO_CFG(100, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
            gpio_set_value(101, 0);
            break;

        case RESUME:
            p_adi752x->main = p_adi752x->main_7B;
            gpio_tlmm_config(GPIO_CFG(100, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
            gpio_set_value(101, 1);
            gpio_set_value(100, 1);
            gpio_set_value(82, 1);
            if(i2c_write_bit(p_adi752x->main,0x41,6,0)) {
                pr_info("[HDMI] Change to 72\n");
                p_adi752x->main = p_adi752x->main_72;
                if(i2c_write_bit(p_adi752x->main,0x41,6,0)) {
                    pr_err("[HDMI] HDMI crash \n");
                    goto setting_err;
                }
            }
            /* Change Package map address */
            i2c_write_byte(p_adi752x->main,0x45,0x66);
            break;

        default:
            break;
    }
    return 0;

setting_err:
    pr_err("[HDMI] %s setting err (%d)\n",__func__,act);
    return -ENXIO;
}



#ifdef CONFIG_HAS_EARLYSUSPEND
void adi752x_early_suspend(struct early_suspend *h)
{
    pr_debug("[HDMI] Enter %s\n",__func__);
    if (p_adi752x->intent_status == HDMI_ACTIVE )
        turn_off_HDMI();

    disable_irq(p_adi752x->main->irq);
    gpio_icPower_setting(SUSPEND);
}


void adi752x_early_resume(struct early_suspend *h)
{
    pr_debug("[HDMI] Enter %s and resume Done\n",__func__);
    p_adi752x->flag_edid_ready = 0;
    gpio_icPower_setting(RESUME);
    schedule_work(&p_adi752x->work);
    enable_irq(p_adi752x->main->irq);
}
#endif

void turn_on_HDMI(void)
{
    if((!p_adi752x->intent_status) || (!p_adi752x->flag_edid_ready)) {
        pr_err("[HDMI] IOCTL_HDMI_START: HDMI not connect\n");
        return;
    }
    if ( !gpio_icPower_setting(HDMI_ACTIVE) ) {

#if USE_AV_MUTE
        /* Set AV Mute */
        i2c_write_byte(p_adi752x->main, 0x4B, 0x40);
        i2c_write_byte(p_adi752x->main, 0x40, 0x80);
        mod_timer(&p_adi752x->av_mute_timer, jiffies + msecs_to_jiffies(AV_MUTE_TIME));
#endif
        /* set LCD related pins driving strength */
        turn_off_lcd();
        i2s_enable_clock();

        fixd_registers();
        p_adi752x->locked_format = setup_audio_video();
        switch_set_state(&hdp->sdev, HDMI_ACTIVE);
    } else {
        pr_err("[HDMI](%s) start ERROR \n",__func__);
        return;
    }

}

int turn_off_HDMI(void)
{
    if (p_adi752x->intent_status == HDMI_ACTIVE)
        i2s_disable_clock();

    if ( 0 != gpio_icPower_setting(HDMI_UNACTIVE) ) {
        pr_err("[HDMI] IOCTL_HDMI_END, Power down ic FAILED! \n");
    }

    p_adi752x->locked_format = LCD_480_800;
    if( p_adi752x->hdmi_status & 0x60 ) {
        switch_set_state(&hdp->sdev, HDMI_PLUGIN);
        init_cec(p_adi752x->main , p_adi752x->cec,  p_adi752x->edid_info);
    } else {
        switch_set_state(&hdp->sdev, NO_PLUGIN);
    }
    if (p_adi752x->ioctl_mode & (1<<1)) {
        p_adi752x->set_tv_format = p_adi752x->tv_format_backup;
        p_adi752x->ioctl_mode = p_adi752x->ioctl_mode & 0xfffffffd;
    }
    turn_on_lcd();

    return 0;
}

void adi752x_detach(struct work_struct *work)
{
    if (p_adi752x->intent_status != HDMI_ACTIVE ) {
        gpio_icPower_setting(HDMI_IC_OFF);
        switch_set_state(&hdp->sdev, NO_PLUGIN);
    } else {
        turn_off_HDMI();
    }
    p_adi752x->flag_edid_ready = 0;
}


void analyse_edid( uint8_t *e)
{
    /* Find out Cea Support Format */
    int tag = 0;
    int block = 0;
    int dtb_base = 0;
    int cea_base = 0;

    int h_a = 0;
    int v_a = 0;
    int h_b = 0;

    int tmp;

    for (block = 0 ; block <=1 ;block++) {
        /* First Block */
        if((p_adi752x->edid_info.segment_current == 1) && (block == 0)) {
            dtb_base = 54;
        } else {
            dtb_base = e[block*0x80 + 2] + block*0x80;
        }

        for(dtb_base = dtb_base; (dtb_base< (((block+1)*0x80) - 18)); dtb_base+=18) {

            if ((e[dtb_base] != 0x0) && (dtb_base != 126)) {
                h_a = (((e[dtb_base+4]&0xf0 )<< 4) | e[dtb_base+2]);
                h_b =(((e[dtb_base+4]&0x0f )<< 8) | e[dtb_base+3]);
                v_a = (((e[dtb_base+7]&0xf0 )<< 4) | e[dtb_base+5]);

                /* 720p 60 */
                if(( h_a == 1280 ) && ( v_a == 720 ) && ( h_b == 370 )) {
                    p_adi752x->edid_info.support_format |= ( 1 << HTX_720p_60 );
                }
                /* 720p 50 */
                if(( h_a == 1280 ) && ( v_a == 720 ) && ( h_b == 700 ) ) {
                    p_adi752x->edid_info.support_format |= ( 1 << HTX_720p_50 );
                }
                /* 480p */
                if(( h_a == 720 ) && ( v_a == 480 ) ) {
                    p_adi752x->edid_info.support_format |= ( 1 << HTX_720_480p ); 
                }
                /* 576p */
                if(( h_a == 720 ) && ( v_a == 576 ) ) {
                    p_adi752x->edid_info.support_format |= ( 1 << HTX_720_576p );
                }
            }
        }

        if(!((p_adi752x->edid_info.segment_current == 1) && (block == 0))) {
            cea_base = block * 0x80 + 4;
            while( cea_base < block * 0x80 +e[block*0x80+2] ) {

                tag = get_bit( e[cea_base], 7, 5 );
                if( tag == 2 ) {

                    for( tmp = cea_base + 1;
                        tmp <= ( cea_base + get_bit(e[cea_base],4,0 ) ); tmp++ ) {

                        switch(get_bit( e[tmp],6,0 )) {
                            case F_640_480p_60:
                                p_adi752x->edid_info.support_format |=
                                                                  ( 1 << HTX_640_480p );
                                break;

                            case F_720_480p_60:
                            case S_720_480p_60:
                                p_adi752x->edid_info.support_format |=
                                                                  ( 1 << HTX_720_480p );
                                break;

                            case S_1280_720p_60:
                                p_adi752x->edid_info.support_format |=
                                                                   ( 1 << HTX_720p_60 );
                                break;

                            case S_1280_720p_50:
                                p_adi752x->edid_info.support_format |=
                                                                   ( 1 << HTX_720p_50 );
                                break;

                            default:
                                break;
                        }
                    }
                } else if ( tag == 3) {
                    p_adi752x->edid_info.hdmi_addr_ab = e[cea_base+4];
                    p_adi752x->edid_info.hdmi_addr_cd = e[cea_base+5];
                    if (((( e[cea_base+1] ) | ( e[cea_base+2] << 8 )) == 0x0C03 ) &&
                          (( e[cea_base+3] ) == 0x00)) {
                            p_adi752x->edid_info.vsdb_hdmi = 1;
                    }
                }
                cea_base += ( get_bit( e[cea_base], 4, 0 ) + 1 );
            }
        }

    }

    pr_info("[HDMI] format = 0x%x\n",p_adi752x->edid_info.support_format);

}

int check_edid( uint8_t* e )
{
    int check_sum = 0;
    int count = 0;

    for (count = 0 ; count < 256 ; count++) {
        check_sum += e[count];
        if (count == 127 && (check_sum & 0xff)) {
            pr_err("[HDMI] check sum fail\n");
            return -1;
        }
        if (count == 255 && ( !check_sum || (check_sum & 0xff))) {
            pr_err("[HDMI] EDID All Zeros or check sum fail\n");
            return -1;
        }
    }

    if(!(( e[0] == 0 ) || ( e[0] == 2 ) || ( e[0] == 0xF0 ) || ( e[0x80] == 0 )
        || ( e[ 0x80 ] == 2 ) || ( e[ 0x80 ] == 0xF0 ) ) ) {
        pr_err( "[HDMI] No valid blocks \n");
        return -1;
    }

    if (!p_adi752x->edid_info.segment_num)
        p_adi752x->edid_info.segment_num = (e[0x7E] /2) +1;

    p_adi752x->edid_info.segment_current++;

    if (p_adi752x->edid_info.segment_num < p_adi752x->edid_info.segment_current) {
        pr_err("[HDMI]segment_num < segment_current \n");
        return -1;
    }
    return 0;
}

void adi752x_read_edid(struct work_struct *work)
{
    if (!p_adi752x->flag_edid_ready) {
        uint8_t edid_dat[256] = {0};
        memset(edid_dat, 0, 256);
        /* read EDID from IC */
        i2c_read(p_adi752x->edid,edid_dat[0],edid_dat,256);
        if ( check_edid(edid_dat) < 0) {
            gpio_icPower_setting(HDMI_IC_OFF);
            gpio_icPower_setting(HDMI_IC_ON);
        } else {
            analyse_edid(edid_dat);
            if(p_adi752x->edid_info.segment_num == p_adi752x->edid_info.segment_current){
                pr_info("[HDMI] edid ready\n");
                if(p_adi752x->intent_status == NO_PLUGIN)
                    switch_set_state(&hdp->sdev, HDMI_PLUGIN);

                init_cec(p_adi752x->main , p_adi752x->cec, p_adi752x->edid_info);
                p_adi752x->flag_edid_ready = 1;
            } else {
                /* EDID segment set */
                i2c_write_byte(p_adi752x->main,0xC4,0x1);
                mod_timer(&p_adi752x->edid_timer,
                           jiffies + msecs_to_jiffies(READ_EDID_TIME));
            }
        }
    }

}

#if USE_AV_MUTE
void adi752x_av_mute(struct work_struct *work)
{
    i2c_write_byte(p_adi752x->main, 0x4B, 0x80);
    i2c_write_byte(p_adi752x->main, 0x40, 0x80);
}

static void adi752x_av_mute_timer(unsigned long unused)
{
        schedule_work(&p_adi752x->av_mute);
}
#endif

static void adi752x_edid_timer(unsigned long unused)
{
    if((!p_adi752x->flag_edid_ready) && (p_adi752x->hdmi_status & 0x60)) {
        schedule_work(&p_adi752x->edid_work);
        mod_timer(&p_adi752x->edid_timer, jiffies + msecs_to_jiffies(POLLING_EDID));
    }
}

static void adi752x_detach_timer(unsigned long unused)
{
    if(!(p_adi752x->hdmi_status & 0x60))
        schedule_work(&p_adi752x->detach);
}


irqreturn_t adi752x_interrupt(int irq, void *dev_id)
{
    schedule_work(&p_adi752x->work);

    return IRQ_HANDLED;
}


void adi752x_interrupt_handle(struct work_struct *work)
{
    uint8_t interrupt_registers[2] = { 0x00 };
    uint8_t clrReg[3] = {0x96,0xff,0xff};
    uint8_t cec_Rx_frame[17] = {0x15,0x0};
    static uint8_t first = 1;

    gpio_icPower_setting(HDMI_IC_ON);

    do{
        /* Read registers */
        if(i2c_read(p_adi752x->main,MAIN_INTERRUPT_BLOCK1,interrupt_registers,2))
            goto interrupt_i2c_err;

        pr_debug("[HDMI] %s, Reg[0x%x] = 0x%x, Reg[0x%x] = 0x%x\n", __func__,
                    MAIN_INTERRUPT_BLOCK1, interrupt_registers[0],
                    MAIN_INTERRUPT_BLOCK2, interrupt_registers[1] );

        /* clear interrupt registers */
        if(i2c_write( p_adi752x->main, clrReg, 3)) {
            pr_err("[HDMI] %s: write IIC err \n", __func__);
            return;
        }
    } while(!gpio_get_value(99));

    /* Read HDMI stauts */
    if(i2c_read(p_adi752x->main,MAIN_STATUS,&p_adi752x->hdmi_status,1))
        goto interrupt_i2c_err;

    pr_info("[HDMI] p_adi752x->hdmi_status = %d\n",p_adi752x->hdmi_status);
    /*  Main interrupt */
    if( interrupt_registers[0] ) {
        if( interrupt_registers[0] & EDID_INT ) {
            schedule_work(&p_adi752x->edid_work);
        }
    }

    /*  Cec interrupt */
    if (interrupt_registers[1]) {
        if ((interrupt_registers[1] & 0x04) && p_adi752x->flag_edid_ready) {
            i2c_read(p_adi752x->cec,cec_Rx_frame[0],cec_Rx_frame,17);
            analyse_msg(cec_Rx_frame);
       }
    }

    if(p_adi752x->intent_status == HDMI_ACTIVE)
        set_audio();

    if( p_adi752x->hdmi_status & 0x60 ) {
        mod_timer(&p_adi752x->edid_timer, jiffies + msecs_to_jiffies(READ_EDID_TIME));
    } else {
        if(first) {
            first = 0;
            schedule_work(&p_adi752x->detach);
        } else
            mod_timer(&p_adi752x->detach_timer, jiffies + msecs_to_jiffies(DETACH_TIME));
    }
    return;
interrupt_i2c_err:
    pr_err("[HDMI] %s i2c_err\n", __func__);
}


/* open command for ADI752X device file */
int adi752x_open(struct inode *inode, struct file *file)
{
    pr_debug("[HDMI] has been opened\n");
    return 0;
}


int adi752x_close(struct inode *inode, struct file *file)
{
    pr_debug("[HDMI] has been closed\n");
    return 0;
}


int adi752x_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    int tmp = 0;

    pr_debug("[HDMI] adi752x_ioctl \n");

    if(_IOC_TYPE(cmd) != IOC_HDMI_ADI752X_MAGIC) {
        pr_err("[HDMI] cmd magic type error\n");
        return -ENOTTY;
    }

    if(_IOC_NR(cmd) > IOC_MAXNR) {
        pr_err("[HDMI]cmd number error, cmd number is %d\n",cmd);
        return -ENOTTY;
    }

    if(_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

    if(err) {
        pr_err("[HDMI] cmd access_ok error\n");
        return -EFAULT;
    }

    switch(cmd) {
        case IOCTL_HDMI_OS_RESOLUTION:
            pr_debug("[HDMI] Enter IOCTL_HDMI_IC_POWERON \n");

            if( p_adi752x->locked_format < HTX_480i || p_adi752x->locked_format > HTX_640_480p)
                p_adi752x->locked_format = LCD_480_800;

            tmp |= os_rsolution[p_adi752x->locked_format*2]<< 16;
            tmp |= os_rsolution[p_adi752x->locked_format*2+1];

            if ( 0 != copy_to_user((int*)arg, &tmp, sizeof(int)) ) {
                pr_err("[HDMI] IOCTL_HDMI_GET_TV_FORMATS: copy_to_user error\n");
                return -EFAULT;
            }
            break;

        case IOCTL_HDMI_GET_TV_FORMATS:
            pr_debug("[HDMI] Enter IOCTL_HDMI_GET_TV_FORMATS \n");

            if ( p_adi752x->flag_edid_ready == 1) {
                tmp = p_adi752x->edid_info.support_format;

                if ( 0 != copy_to_user((int*)arg, &tmp, sizeof(tmp)) ) {
                    pr_err("[HDMI] IOCTL_HDMI_GET_TV_FORMATS: copy_to_user error\n");
                    return -EFAULT;
                }
            }
            else
                return -EFAULT;

            break;

        case IOCTL_HDMI_SET_TV_FORMATS:
            pr_info("[HDMI] Enter IOCTL_HDMI_SET_TV_FORMATS ( 0x%x ) \n" , (int)arg);

            p_adi752x->set_tv_format = HTX_720p_50;

            if ( ( (int)arg > 8) || ( (int)arg < 0) ) {
                pr_err("[HDMI] input value out of range: [%d]\n", (int)arg);
                return -EINVAL;
            } else{
                p_adi752x->set_tv_format = (int)arg;
                pr_debug("[HDMI] set_tv_format = %d\n", p_adi752x->set_tv_format);
            }
            break;

        case IOCTL_HDMI_START:
            pr_info("[HDMI] Enter IOCTL_HDMI_START \n");
            turn_on_HDMI();
            if ( 0 != copy_to_user((int*)arg, &p_adi752x->locked_format,
                sizeof(p_adi752x->locked_format)) ) {
                pr_err("[HDMI] IOCTL_HDMI_START: copy_to_user error\n");
            }
            break;

        case IOCTL_HDMI_END:
            pr_info("[HDMI] Enter IOCTL_HDMI_END \n");
            turn_off_HDMI();
            if ( 0 != copy_to_user((int*)arg, &p_adi752x->locked_format,
                sizeof(p_adi752x->locked_format)) ) {
               pr_err("[HDMI] IOCTL_HDMI_END: copy_to_user error\n");
            }
            break;

        case IOCTL_HDMI_GET_MODE:
            pr_debug("[HDMI] Enter IOCTL_HDMI_GET_MODE \n");
            if ( 0 != copy_to_user((int*)arg, &p_adi752x->ioctl_mode,
                       sizeof(p_adi752x->ioctl_mode)) ) {
                pr_err("[HDMI] IOCTL_HDMI_START: copy_to_user error\n");
            }
            break;

        case IOCTL_HDMI_SET_MODE:
            pr_debug("[HDMI] Enter IOCTL_HDMI_SET_MODE \n");
            p_adi752x->ioctl_mode = (int)arg;
            if (p_adi752x->ioctl_mode & (1<<1))
                p_adi752x->tv_format_backup = p_adi752x->set_tv_format;
            break;

        case IOCTL_HDMI_GET_CURRENT_FORMAT:
            if ( 0 != copy_to_user((int*)arg, &p_adi752x->set_tv_format,
                       sizeof(p_adi752x->set_tv_format) )) {
                pr_err("[HDMI] IOCTL_HDMI_START: copy_to_user error\n");
            }
            break;

        default:
            break;
    }

    return 0;
}

int adi752x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_adapter *edid_adapter = i2c_get_adapter(0);
    struct i2c_adapter *cec_adapter = i2c_get_adapter(0);
    struct i2c_adapter *main_72_adapter = i2c_get_adapter(0);
    p_adi752x = kzalloc(sizeof(struct adi752x_data), GFP_KERNEL);
    if (p_adi752x == NULL)
        goto probe_err;

    hdp = kzalloc(sizeof(struct hdmi_res), GFP_KERNEL);
    if (!hdp)
        goto probe_err;

    p_adi752x->main = client;
    p_adi752x->main_7B= client;
    p_adi752x->main_72= i2c_new_dummy(main_72_adapter,0x39);
    p_adi752x->main_72->irq = p_adi752x->main->irq;
    p_adi752x->edid = i2c_new_dummy(edid_adapter,0x3F);
    p_adi752x->cec = i2c_new_dummy(cec_adapter,0x3C);
    /* Detect the HDP status */
    hdp->sdev.name = "acer-hdmi";
    hdp->sdev.print_name = acer_hdmi_print_name;
    hdp->sdev.print_state = acer_hdmi_print_state;

    p_adi752x->set_tv_format = HTX_720p_50;
    p_adi752x->locked_format = LCD_480_800;
    p_adi752x->flag_edid_ready = 0;

    if (switch_dev_register(&hdp->sdev)) {
        goto probe_err;
    }

    /* check i2c functionality is workable */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("[HDMI] i2c_check_functionality error!\n");
        goto i2c_check_err;
    }
    gpio_icPower_setting(INIT);
    strlcpy(client->name, ADI752x_DRIVER_NAME, I2C_NAME_SIZE);
    i2c_set_clientdata(client, p_adi752x);

    INIT_WORK(&p_adi752x->work, adi752x_interrupt_handle);
    INIT_WORK(&p_adi752x->edid_work, adi752x_read_edid);
    INIT_WORK(&p_adi752x->detach, adi752x_detach);
#if USE_AV_MUTE
    INIT_WORK(&p_adi752x->av_mute, adi752x_av_mute);
#endif
    if (client->irq) {
        if ( request_irq(client->irq, adi752x_interrupt,
                         IRQF_TRIGGER_FALLING, ADI752x_DRIVER_NAME, p_adi752x) ) {
            /* request failed */
            pr_err("[HDMI] irq failed! \n");
            goto request_irq_err;
        }
    }

    /* register misc device */
    if (misc_register(&adi752x_dev)) {
        pr_err("[HDMI]:adi752x_dev register misc device failed!");
        goto register_misc_err;
    }
    setup_timer(&p_adi752x->edid_timer, adi752x_edid_timer, 0);
    setup_timer(&p_adi752x->detach_timer, adi752x_detach_timer, 0);
#if USE_AV_MUTE
    setup_timer(&p_adi752x->av_mute_timer, adi752x_av_mute_timer, 0);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    p_adi752x->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
    p_adi752x->early_suspend.suspend = adi752x_early_suspend;
    p_adi752x->early_suspend.resume = adi752x_early_resume;
    register_early_suspend(&p_adi752x->early_suspend);
#endif
    pr_info("[HDMI] probe done \n");

    return 0;

register_misc_err:
    free_irq(client->irq, p_adi752x);
request_irq_err:
    kfree(p_adi752x);
i2c_check_err:
    switch_dev_unregister(&hdp->sdev);
probe_err:
    pr_err("[HDMI] probe error\n");
    return -ENOTSUPP;
}

int adi752x_remove(struct i2c_client *client)
{
    free_irq(client->irq, p_adi752x);
    kfree(p_adi752x);
    switch_dev_unregister(&hdp->sdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&p_adi752x->early_suspend);
#endif

    return 0;
}

static const struct i2c_device_id adi752x_id[] = {
    { ADI752x_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver adi752x_driver = {
    .probe        = adi752x_probe,
    .remove       = adi752x_remove,
    .id_table     = adi752x_id,
    .driver       = {
        .name  = ADI752x_DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

void __exit adi752x_exit(void)
{
    i2c_unregister_device(p_adi752x->cec);
    i2c_unregister_device(p_adi752x->edid);
    i2c_del_driver(&adi752x_driver);
}

int __init adi752x_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&adi752x_driver);
    if (ret)
        pr_err("[HDMI]%s, i2c_add_driver failed!\n", __func__);

    return ret;
}

module_init(adi752x_init);
module_exit(adi752x_exit);

MODULE_AUTHOR("Allan Lin <Allan_Lin@acer.com.tw>");
MODULE_DESCRIPTION("ADI ADV752X driver");
