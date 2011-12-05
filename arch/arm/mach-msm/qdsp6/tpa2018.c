/* First release: 2009.07.08
 * TI's TPA2018 is an amp.s
 */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <mach/tpa2018.h>
#include <mach/board.h>

#if 0
#define ACER_DBG(fmt, arg...) printk(KERN_INFO "[TPA2051]: %s: " fmt "\n", __FUNCTION__, ## arg)
#else
#define ACER_DBG(fmt, arg...) do {} while (0)
#endif

#define ACER_TPA_DEBUG -1

#define TPA2018_DRIVER_NAME "tpa2018"

/* Stream Type definition */
#define STREAM_VOICE_CALL	0
#define STREAM_SYSTEM	1
#define STREAM_RING	2
#define STREAM_MUSIC	3
#define STREAM_ALARM	4
#define STREAM_NOTIFICATION	5
#define STREAM_BLUETOOTH_SCO	6

/* Polling volue for debug */
#if 0
#define POLLING_FOR_DEBUG
#endif
#ifdef POLLING_FOR_DEBUG
#define POLLING_TIME    1000 /* milliseconds */
#endif

static int __init tpa2018_init(void);
static int tpa2018_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpa2018_remove(struct i2c_client *client);
static int tpa2018_open(struct inode *inode, struct file *file);
static int tpa2018_close(struct inode *inode, struct file *file);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static void tpa2018_arg_init(void);
static int tpa2018_check_gpio_and_regvalue(void);
static int tpa2018_set_limitor(int type);
static int tpa2018_speaker_switch(int command);
static int tpa2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int tpa2018_suspend(struct i2c_client *client, pm_message_t mesg);
static int tpa2018_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpa2018_early_suspend(struct early_suspend *h);
static void tpa2018_early_resume(struct early_suspend *h);
#endif
#ifdef POLLING_FOR_DEBUG
static struct work_struct show2051_wq;
static void show_tpa2501(struct work_struct *work);
static void polling_timer_func(unsigned long unused);
#endif

static void software_shutdown_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(en_speaker_wq1, software_shutdown_work);

static int adie_act_flag;
static bool tpa_act_flag;
static bool hs_state;
static int hs_vol;
static bool mute_state;
static u32 FM_vol;
static u32 FM_headset_volume[16] = {0x0, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xF, 0x11, 0x13, 0x15, 0x17, 0x19};
static u32 FM_speaker_volume[16] = {0x0, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xF, 0x11, 0x13, 0x15};

static const struct i2c_device_id tpa2018_id[] = {
	{ TPA2018_DRIVER_NAME, 0 },
	{ }
};

static struct tpa2018_data {
	struct i2c_client *client;
	wait_queue_head_t wait;
	//int def_vol;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef POLLING_FOR_DEBUG
	struct timer_list polling_timer;
#endif
} tpa2018_data;

static const struct file_operations tpa2018_fops = {
	.owner      = THIS_MODULE,
	.open       = tpa2018_open,
	.release    = tpa2018_close,
	.ioctl      = tpa2018_ioctl,
};

static struct i2c_driver tpa2018_driver = {
	.probe		= tpa2018_probe,
	.remove		= tpa2018_remove,
	.id_table	= tpa2018_id,
	.suspend	= tpa2018_suspend,
	.resume		= tpa2018_resume,
	.driver		= {
	.name = TPA2018_DRIVER_NAME,
	},
};

static struct miscdevice tpa2018_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TPA2018_DRIVER_NAME,
	.fops = &tpa2018_fops,
};

static int i2c_read(struct i2c_client *client, char *buf, int count){
	if(1 != i2c_master_send(client, buf, 1)){
		pr_err("[TPA2018] i2c_read --> Send reg. info error\n");
		return -1;
	}

	if(count != i2c_master_recv(client, buf, count)){
		pr_err("[TPA2018] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

static int i2c_write(struct i2c_client *client, char *buf, int count){
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[TPA2018] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

static void tpa2018_arg_init(void)
{
	int count;
	uint8_t tpa_spkr_wBuf[7]={0X00, 0xC2, 0x21, 0x54, 0x0D, 0x8D, 0xAD};
	uint8_t tpa_hs_wBuf[7]={0X00, 0x4C, 0x21, 0x54, 0x6D, 0xCD, 0xAD};
	uint8_t tpa_rBuf[7]={0};

	msleep(10);
#if 1
	if (hs_state == 1) {
		for (count = 0; count < 7; count++)
			tpa2018_set_control(1, count, tpa_hs_wBuf[count]);
	} else {
		for (count = 0; count < 7; count++)
			tpa2018_set_control(1, count, tpa_spkr_wBuf[count]);
	}
#endif

	pr_info("[TPA2051] init - [");
	for(count = 0; count < 7; count++) {
		tpa_rBuf[count] = tpa2018_set_control(2, count, 0);
		pr_info("0x%x, ", tpa_rBuf[count]);
	}
	pr_info("]\n");
}

static int tpa2018_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;

	tpa2018_data.client = client;

	/* spk_amp_en - speaker amplifier enable*/
	res = gpio_request(SPK_AMP_EN, "SPK AMP EN");
	if (res) {
		pr_err("GPIO request for SPK AMP EN failed!\n");
		goto gpio_err;
	}
	gpio_set_value(SPK_AMP_EN, 1);

	pr_debug("[TPA2018] Probe!!\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[TPA2018] i2c_check_functionality error!\n");
		return -ENOTSUPP;
	}
	strlcpy(client->name, TPA2018_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &tpa2018_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	tpa2018_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	tpa2018_data.early_suspend.suspend = tpa2018_early_suspend;
	tpa2018_data.early_suspend.resume = tpa2018_early_resume;
	register_early_suspend(&tpa2018_data.early_suspend);
#endif

#ifdef POLLING_FOR_DEBUG
	INIT_WORK(&show2051_wq, show_tpa2501);
	setup_timer(&tpa2018_data.polling_timer, polling_timer_func, 0);
	mod_timer(&tpa2018_data.polling_timer, jiffies + msecs_to_jiffies(POLLING_TIME));
#endif

	res = misc_register(&tpa2018_dev);
	if (res) {
		pr_err("tpa2018_probe: tpa2018_dev register failed\n");
		goto error_tpa2018_dev;
	}

	mute_state = false;
	tpa2018_arg_init();

	pr_info("[TPA2018] probe done\n");
	return 0;

gpio_err:
	gpio_free(SPK_AMP_EN);
error_tpa2018_dev:
	pr_err("[TPA2018] probe: tpa2018_dev error\n");
	return res;
}

static int tpa2018_remove(struct i2c_client *client)
{
	ACER_DBG("remove tpa2018\n");
	gpio_free(SPK_AMP_EN);
	return 0;
}

static int tpa2018_open(struct inode *inode, struct file *file)
{
	pr_debug("[TPA2018] has been opened\n");
	return 0;
}

static int tpa2018_close(struct inode *inode, struct file *file)
{
	pr_debug("[TPA2018] has been closed\n");
	return 0;
}

static int tpa2018_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("[TPA2018] low power suspend init done.\n");

	return 0;
}

static int tpa2018_resume(struct i2c_client *client)
{
	pr_debug("[TPA2018] normal resume init done.\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpa2018_early_suspend(struct early_suspend *h)
{
	int reg1 = 0;
	ACER_DBG("tpa2018_early_suspend +++\n");
	pr_debug("[TPA2018] %s ++ entering\n", __FUNCTION__);
	//Recover after MWC.
#if 1
	reg1 = tpa2018_set_control(2, 1, 0);
	if (adie_act_flag == 0) {
		if (!(reg1 & TPA2051_SWS)) {
			tpa2018_software_shutdown(1);
			ACER_DBG("tpa2018_software_shutdown(1)!!\n");
		}
	}
#endif
	pr_debug("[TPA2018] %s -- leaving\n", __FUNCTION__);
	ACER_DBG("tpa2018_early_suspend --- \n");
}

static void tpa2018_early_resume(struct early_suspend *h)
{
	ACER_DBG("tpa2018_early_resume +++ \n");
	pr_debug("[TPA2018] %s ++ entering\n", __FUNCTION__);
	//Recover after MWC.
	if (adie_act_flag == 0) {
		if (!gpio_get_value(SPK_AMP_EN)) {
			gpio_set_value(SPK_AMP_EN, 1);
			ACER_DBG("SPK_AMP_EN pull low!!\n");
		}
		tpa2018_arg_init();
		schedule_delayed_work(&en_speaker_wq1, 100);
	}
	pr_debug("[TPA2018] %s -- leaving\n", __FUNCTION__);
	ACER_DBG("tpa2018_early_resume --- \n");
}
#endif

#ifdef POLLING_FOR_DEBUG
static void show_tpa2501(struct work_struct *work)
{
	int count;
	uint8_t tpa_rBuf[7]={0};

	for (count = 0; count < 7; count++)
		tpa_rBuf[count] = tpa2018_set_control(2, count, 0);
	pr_info("[TPA2051] - [1]0x%x, [2]0x%x, [3]0x%x, [4]0x%x, [5]0x%x, [6]0x%x \n",
		tpa_rBuf[1], tpa_rBuf[2], tpa_rBuf[3], tpa_rBuf[4], tpa_rBuf[5], tpa_rBuf[6]);
}

static void polling_timer_func(unsigned long unused)
{
	schedule_work(&show2051_wq);
	mod_timer(&tpa2018_data.polling_timer, jiffies +
	msecs_to_jiffies(POLLING_TIME));
}
#endif

static int tpa2018_check_gpio_and_regvalue(void)
{
	int count;
	uint8_t tpa_rBuf[7]={0};

	if (ACER_TPA_DEBUG) { pr_info("[TPA2051] argu - ["); }
	for(count=0;count<7;count++) {
		tpa_rBuf[count] = tpa2018_set_control(2, count, 0);
		if (ACER_TPA_DEBUG) { pr_info("0x%x, ", tpa_rBuf[count]); }
	}
	if (ACER_TPA_DEBUG) { pr_info("]\n"); }

	if (gpio_get_value(SPK_AMP_EN) == 0) {
		gpio_set_value(SPK_AMP_EN, 1);
		tpa2018_arg_init();
		return 0;
	}

	return 0;
}

static int tpa2018_set_limitor(int type)
{
	switch(type) {
		case STREAM_VOICE_CALL:
			return 0;

		case STREAM_SYSTEM:
			return 0;

		case STREAM_RING:
			return 0;

		case STREAM_MUSIC:
			return 0;

		case STREAM_ALARM:
			return 0;

		case STREAM_NOTIFICATION:
			return 0;

		default:
			return 0;
	}
}

void set_adie_flag(int flag)
{
	ACER_DBG("adie flag = %d \n", flag);
	adie_act_flag = flag;
}

int get_adie_flag(void)
{
	int flag;
	flag = adie_act_flag;
	ACER_DBG("adie flag = %d \n", flag);
	return flag;
}

int tpa2018_mute(int command)
{
	int reg4temp = 0;

	reg4temp = tpa2018_set_control(2, 4, 0);
	if (command == 1) {
		ACER_DBG("mute state = true \n");
		mute_state = true;
		reg4temp |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, reg4temp);
	} else {
		ACER_DBG("mute state = false \n");
		mute_state = false;
		if (hs_state)
			tpa2018_headset_switch(1);
		else
			tpa2018_speaker_switch(1);;
	}

	return 0;
}

static void software_shutdown_work(struct work_struct *work)
{
	int reg1temp = 0;
	if (adie_act_flag) {
		pr_info("[TPA2051] resume_Enable Speaker AMP \n");
		reg1temp = tpa2018_set_control(2, 1, 0);
		reg1temp &= ~TPA2051_SWS;
		tpa2018_set_control(1, 1, reg1temp);
		if (hs_state == 1)
			tpa2018_set_control(1, 4, 0x6D);
		else
			tpa2018_set_control(1, 4, 0xD);
	} else {
		tpa2018_software_shutdown(1);
		pr_info("[TPA2051] resume_Disable Speaker AMP \n");
	}
}

int tpa2018_software_shutdown(int command)
{
	int reg1temp = 0;
#if 1
	if (tpa_act_flag)
		return 0;
#endif

	tpa2018_check_gpio_and_regvalue();

	reg1temp = tpa2018_set_control(2, 1, 0);
	if (command == 1) {
		ACER_DBG("software shutdown = true \n");
		reg1temp |= TPA2051_SWS;
	} else {
		ACER_DBG("software shutdown = false \n");
		reg1temp &= ~TPA2051_SWS;
	}
	tpa2018_set_control(1, 1, reg1temp);

	return 0;
}

int spkr_amp(int command)
{
	if (command == 1) {
		tpa2018_speaker_switch(1);
	} else {
		tpa2018_speaker_switch(0);
	}
	return 0;
}

static int tpa2018_speaker_switch(int command)
{
	int reg1temp = 0, reg4temp = 0;
	if (command == 1) {
		reg4temp = tpa2018_set_control(2, 4, 0);
		reg4temp |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, reg4temp);

		reg1temp = tpa2018_set_control(2, 1, 0);
		reg1temp |= TPA2051_LIMI_EN;
		reg1temp |= TPA2051_LIMI_SEL;
		reg1temp &= ~TPA2051_VMBYPASS;
		reg1temp &= ~TPA2051_SWS;
		reg1temp |= TPA2051_SPKR_EN;
		tpa2018_set_control(1, 1, reg1temp);

		reg4temp = tpa2018_set_control(2, 4, 0);
		if (!mute_state) {
			reg4temp &= ~TPA2051_MUTE;
		} else {
			reg4temp |= TPA2051_MUTE;
		}
		tpa2018_set_control(1, 4, reg4temp);
		tpa2018_set_control(1, 6, 0xAD);
		ACER_DBG("tpa2018_headset_switch = true \n");
	} else {
		reg4temp = tpa2018_set_control(2, 4, 0);
		reg4temp |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, reg4temp);

		reg1temp = tpa2018_set_control(2, 1, 0);
		reg1temp &= ~TPA2051_HS_EN;
		reg1temp &= ~TPA2051_SPKR_EN;
		reg1temp |= TPA2051_SWS;
		tpa2018_set_control(1, 1, reg1temp);

		hs_vol = tpa2018_set_control(2, 4, 0);
		hs_vol &= ~TPA2051_MUTE;
		hs_vol |= TPA2051_SPK_GAIN;
		tpa2018_set_control(1, 5, hs_vol);
		ACER_DBG("tpa2018_headset_switch = false \n");
	}
	return 0;
}

int tpa2018_set_headset_state(bool en)
{
	pr_info("tpa2018_set_headset_state() enable=%d \n",en);
	if(en == 1) {
		hs_state = 1;
	} else {
		hs_state = 0;
	}
	return 0;
}

bool tpa2018_get_headset_state()
{
	pr_info("tpa2018_get_headset_state() hs_state=%d \n",hs_state);
	return hs_state;
}

int tpa2018_headset_switch(int command)
{
	int reg1temp = 0, reg4temp = 0, reg5temp = 0;
	if(command == 1){
		reg4temp = tpa2018_set_control(2, 4, 0);
		reg4temp |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, reg4temp);

		reg1temp = tpa2018_set_control(2, 1, 0);
		reg1temp |= TPA2051_LIMI_EN;
		reg1temp &= ~TPA2051_LIMI_SEL;
		reg1temp &= ~TPA2051_SWS;
		reg1temp |= TPA2051_HS_EN;
		reg1temp &= ~TPA2051_SPKR_EN;
		reg1temp &= ~TPA2051_VMBYPASS;
		tpa2018_set_control(1, 1, reg1temp);

		reg4temp = tpa2018_set_control(2, 4, 0);
		if (!mute_state) {
			reg4temp &= ~TPA2051_MUTE;
			reg4temp |= TPA2051_HS;
		} else {
			reg4temp |= TPA2051_MUTE;
		}
		tpa2018_set_control(1, 4, reg4temp);

		reg5temp = tpa2018_set_control(2, 5, 0);
		reg5temp |= TPA2051_HS_ODB;
		tpa2018_set_control(1, 5, reg5temp);
		tpa2018_set_control(1, 6, 0xAD);
		ACER_DBG("tpa2018_headset_switch = true \n");
	}
	else {
		reg4temp = tpa2018_set_control(2, 4, 0);
		reg4temp |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, reg4temp);

		reg1temp = tpa2018_set_control(2, 1, 0);
		reg1temp &= ~TPA2051_HS_EN;
		reg1temp |= TPA2051_SWS;
		tpa2018_set_control(1, 1, reg1temp);

		hs_vol = tpa2018_set_control(2, 5, 0);
		hs_vol |= TPA2051_MUTE;
		tpa2018_set_control(1, 4, hs_vol);
		ACER_DBG("tpa2018_headset_switch = false \n");
	}

	return 0;
}

int tpa2018_set_control(int commad, int regiter, int value)
{
	uint8_t tpa_wBuf[2];
	uint8_t tpa_rBuf;

	struct i2c_client *client = tpa2018_data.client;

	switch(commad){
		case 1:
			tpa_wBuf[0] = regiter;
			tpa_wBuf[1] = value;
			i2c_write(client, tpa_wBuf, 2);
			ACER_DBG("[TPA2018] WRITE GAIN CONTROL regiter=%d, value=0x%x\n",regiter,value);
			msleep(1);
			return 0;

		case 2:
			tpa_rBuf = regiter;
			i2c_read(client, &tpa_rBuf, 1);
			ACER_DBG("[TPA2018] READ GAIN CONTROL \n");
			msleep(1);
			return tpa_rBuf;

		default:
			pr_err("[TPA2018]: Command not found!\n");
			return -1;
	}
}

static int tpa2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 uparam;
	int reg3temp = 0;

	struct i2c_client *client = tpa2018_data.client;

	pr_debug("[TPA2018] tpa2018 ioctl \n");
	if(_IOC_TYPE(cmd) != TPA2018_IOCTL_MAGIC){
		pr_err("[TPA2018] IOCTL: cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("[TPA2018] IOCTL: cmd number error\n");
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_NONE)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("[TPA2018] IOCTL: cmd access right error\n");
		return -EFAULT;
	}
	if( client == NULL){
		pr_err("[TPA2018] IOCTL: I2C driver not install (tpa2018_ioctl)\n");
		return -EFAULT;
	}

	switch(cmd){
		case TPA2018_SET_FIXED_GAIN:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;
			ACER_DBG(" uparam = %d.\n", uparam);
			if (hw_version <= 3) {
				uparam*=20;
				ACER_DBG(" uparam*=20 %d.\n", uparam);
			} else {
				uparam*=26;
				ACER_DBG(" uparam*=26 %d.\n", uparam);
			}
			uparam/=100;
			ACER_DBG(" uparam/=100 = %d.\n", uparam);
			return 0;

		case TPA2018_SET_STREAM_TYPE:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;
			ACER_DBG(" Stream Type = %d.\n", uparam);
			tpa2018_set_limitor(uparam);
			return 0;

		case TPA2018_FM_HS_VOLUME_CONTROL:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;

			//FM adjust volume timing is close to switching device. So, we add a delay for timeing issue.
			msleep(50);

			reg3temp = tpa2018_set_control(2, 3, 0);
			reg3temp &= ~(31U << 3);
			reg3temp |= (1U << 3);
			tpa2018_set_control(1, 3, reg3temp);
			reg3temp = tpa2018_set_control(2, 3, 0);

			FM_vol = uparam;
			uparam/=10;
			uparam = FM_headset_volume[uparam];
			uparam |= TPA2051_HS_ODB;

			tpa2018_set_control(1, 5, uparam);
			hs_vol = uparam;
			ACER_DBG("FM HS Volume = %d.\n", uparam);
			tpa2018_set_headset_state(1);
			return 0;

		case TPA2018_FM_SPK_VOLUME_CONTROL:
			if (copy_from_user(&uparam, (void *)arg,
				sizeof(uparam)))
				return -1;

			//FM adjust volume timing is close to switching device. So, we add a delay for timeing issue.
			msleep(50);

			reg3temp = tpa2018_set_control(2, 3, 0);
			reg3temp &= ~(31U << 3);
			reg3temp |= (1U << 3);
			tpa2018_set_control(1, 3, reg3temp);
			reg3temp = tpa2018_set_control(2, 3, 0);

			FM_vol = uparam;
			uparam/=10;
			uparam = FM_speaker_volume[uparam];

			tpa2018_set_control(1, 4, uparam);
			uparam |= TPA2051_SPK_GAIN;
			tpa2018_set_control(1, 5, uparam);
			ACER_DBG("FM SPK Volume = %d.\n", uparam);
			tpa2018_set_headset_state(0);
			return 0;

		case TPA2018_FM_RESET_VOLUME:
			tpa2018_arg_init();
			ACER_DBG("Reset TPA2018 Volume = %d.\n", uparam);
			return 0;

		case TPA2018_OPEN:
			tpa2018_software_shutdown(0);
			tpa_act_flag = true;
			return 0;

		case TPA2018_CLOSE:
			if (tpa_act_flag)
				tpa2018_software_shutdown(1);
			tpa_act_flag = false;
			return 0;

		case TPA2018_GET_HEADSET_STATUS: {
			bool status;
			memset(&status, 0, sizeof(status));
			status = tpa2018_get_headset_state();
			if (copy_to_user((void*) arg, &status, sizeof(status)))
			    return -EFAULT;

			return 0;
		}
		case TPA2018_SET_SPEAKER_SWITCH: {
			if (copy_from_user(&uparam, (void *)arg, sizeof(uparam)))
				return -1;
			ACER_DBG(" speaker enable = %d.\n", uparam);
			tpa2018_speaker_switch((int)uparam);
			return 0;
		}
		case TPA2018_GET_FM_VOLUME: {
			if (copy_to_user((void*) arg, &FM_vol, sizeof(FM_vol)))
			    return -EFAULT;

			return 0;
		}
		default:
			pr_err("[TPA2018] IOCTL: Command not found!\n");
			return -1;
	}
}

static void __exit tpa2018_exit(void)
{
	i2c_del_driver(&tpa2018_driver);
}

static int __init tpa2018_init(void)
{
	int res=0;

	res = i2c_add_driver(&tpa2018_driver);
	if (res){
		pr_err("[TPA2018]i2c_add_driver failed! \n");
		return res;
	}

	pr_info("[TPA2018] tpa2018 device init ok!\n");
	return 0;
}

module_init(tpa2018_init);
module_exit(tpa2018_exit);

MODULE_AUTHOR("Andyl Liu <Andyl_Liu@acer.com.tw>");
MODULE_DESCRIPTION("TPA2018-380 driver");
