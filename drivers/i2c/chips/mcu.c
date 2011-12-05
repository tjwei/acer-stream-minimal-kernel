/*
 * First release: 2009.11.15
 * - IOCTL for led
 * - IOCTL for battery
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
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/power_supply.h>
#include <mach/mcu.h>

#define DEV_IOCTLID               0x11
#define IOC_MAXNR                 15
#define IOCTL_SET_RED_LED         _IOW(DEV_IOCTLID, 1, int)
#define IOCTL_SET_GREEN_LED       _IOW(DEV_IOCTLID, 2, int)
#define IOCTL_SET_WHITE_LED       _IOW(DEV_IOCTLID, 3, int)
#define IOCTL_SET_KEY_LED         _IOW(DEV_IOCTLID, 4, int)
#define IOCTL_GET_AVE_CUTTENT     _IOW(DEV_IOCTLID, 5, int)
#define IOCTL_SET_MISSING_CALL_LED  _IOW(DEV_IOCTLID, 6, int)
#define IOCTL_SET_LOW_BATTERY_LED   _IOW(DEV_IOCTLID, 7, int)
#define IOCTL_SET_CRITICAL_LOW_LED  _IOW(DEV_IOCTLID, 8, int)
#define IOCTL_KPD_LED_ON          _IOW(DEV_IOCTLID, 9, int)
#define IOCTL_KPD_LED_OFF         _IOW(DEV_IOCTLID, 10, int)
#define IOCTL_GET_MCU_VERSION     _IOW(DEV_IOCTLID, 11, int)
#define IOCTL_SET_CHARGE_CONTROL  _IOW(DEV_IOCTLID, 12, int)
#define IOCTL_MCU_WRITE_DATA      _IOW(DEV_IOCTLID, 13, int)
#define IOCTL_MCU_READ_DATA       _IOW(DEV_IOCTLID, 14, int)

#define MCU_DRIVER_NAME           "mcu"

static int __init mcu_init(void);
static int mcu_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mcu_remove(struct i2c_client *client);
static int mcu_suspend(struct i2c_client *client, pm_message_t mesg);
static int mcu_resume(struct i2c_client *client);
static irqreturn_t mcu_interrupt(int irq, void *dev_id);
static void mcu_work_func(struct work_struct *work);
static int mcu_open(struct inode *inode, struct file *file);
static int mcu_close(struct inode *inode, struct file *file);
static int mcu_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int i2c_write(struct i2c_client *client, char *buf);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static int mcu_write(struct i2c_client *client,char reg, char cmd);
void mcu_dark_mode(void);
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
static void led_on(void);
static void led_off(void);
#endif
static void batt_info_update(void);
static void mcu_power_mode(char mode);
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
static void mcu_led_work_func(struct work_struct *work);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mcu_early_suspend(struct early_suspend *h);
static void mcu_early_resume(struct early_suspend *h);
#endif
static struct fuel_data *buf_data;
static struct mutex mcu_mutex;
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
static struct delayed_work led_wq;
static bool mcu_reg = false;
static bool screen_off_flag = false;
void kpd_led_on(void);
void kpd_led_off(void);
#endif

static __u8 last_level;
static __s16 last_temp;
static __u8 Count = 0;
int led_flag=0;
int check_led=0;
int fake_resume=0;
static struct wake_lock work_wake_lock;

static const struct i2c_device_id mcu_id[] = {
  { MCU_DRIVER_NAME, 0 },
  { }
};

/* Data for I2C driver */
static struct mcu_data {
  struct i2c_client *client;
  struct input_dev *input;
  struct work_struct work;
  wait_queue_head_t wait;
#ifdef CONFIG_HAS_EARLYSUSPEND
  struct early_suspend early_suspend;
#endif
  unsigned long last_jiffies;
  int prekey;
} mcu_data;

/*File operation of MCU device file */
static const struct file_operations mcu_fops = {
  .owner     = THIS_MODULE,
  .open      = mcu_open,
  .release   = mcu_close,
  .ioctl     = mcu_ioctl,
};

/* new style I2C driver struct */
static struct i2c_driver mcu_driver = {
  .probe     = mcu_probe,
  .remove    = mcu_remove,
  .id_table  = mcu_id,
  .suspend   = mcu_suspend,
  .resume    = mcu_resume,
  .driver    = {
    .name      = MCU_DRIVER_NAME,
  },
};

static struct miscdevice mcu_dev = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = MCU_DRIVER_NAME,
  .fops = &mcu_fops,
};

static int __init mcu_init(void)
{
  int res=0;

  res = i2c_add_driver(&mcu_driver);

  if (res){
    pr_err("[MCU]i2c_add_driver failed! \n");
    return res;
  }

  return 0;
}

static int mcu_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int err = 0;
  uint8_t battery_buf[10] = {0};

  mcu_data.client = client;
  mcu_data.prekey = 1;
  mcu_data.last_jiffies = 0;

  pr_debug("[MCU] %s ++ entering\n", __FUNCTION__);
  buf_data = kzalloc(sizeof(*buf_data), GFP_KERNEL);
  if (!buf_data)
      return -ENOMEM;

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    pr_err("[MCU] i2c_check_functionality error!\n");
    return -ENOTSUPP;
  }
  strlcpy(client->name, MCU_DRIVER_NAME, I2C_NAME_SIZE);
  i2c_set_clientdata(client, &mcu_data);

  mutex_init(&mcu_mutex);

  INIT_WORK(&mcu_data.work, mcu_work_func);
  init_waitqueue_head(&mcu_data.wait);

  wake_lock_init(&work_wake_lock, WAKE_LOCK_SUSPEND,MCU_DRIVER_NAME);
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
  INIT_DELAYED_WORK(&led_wq, mcu_led_work_func);
#endif

  battery_buf[0] = MCU_POWER_STATUS;
  battery_buf[1] = 0;
  i2c_read(client, battery_buf, 10);
  if(battery_buf[0]&MCU_EXT_POWER_OUT)
  {
     buf_data->charger_type = ACER_CHARGER_TYPE_NO_CHARGER;
     pr_info("[MCU] probe CABLE-OUT\n");
  }else{
     if(battery_buf[0]&MCU_AC_CABLE){
            buf_data->charger_type = ACER_CHARGER_TYPE_IS_AC;
     }else{
            mcu_write(mcu_data.client,MCU_USB_CHARGE,MCU_USB_CHARGE_500MA);
            buf_data->charger_type = ACER_CHARGER_TYPE_IS_USB;
     }
  }
  if(battery_buf[0]&MCU_NO_CHARGING)
  {
     buf_data->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  else{
     buf_data->charge_status = POWER_SUPPLY_STATUS_CHARGING;
  }
  buf_data->capacity = battery_buf[1];
  last_level = battery_buf[1];
  buf_data->temperature = ((int16_t)battery_buf[4]) * ((battery_buf[5]&0x80)?-10:10);
  last_temp = buf_data->temperature;
  buf_data->voltage = (((uint16_t)battery_buf[7])<<8)|battery_buf[6];
  buf_data->health = POWER_SUPPLY_HEALTH_GOOD;
  buf_data->have_battery = 1;
  fuel_update(*buf_data);

  /* Enable MCU chip by gpio 39. */
  if (client->irq) {
    err = request_irq(client->irq, mcu_interrupt, IRQF_TRIGGER_RISING,
          MCU_DRIVER_NAME, &mcu_data);
    if (err < 0) {
      pr_err("[MCU] request_irq error! %d\n", err);
      free_irq(client->irq, &mcu_data);
    }
    else{
      enable_irq_wake(client->irq);
    }
  }

  err = misc_register(&mcu_dev);
  if (err) {
    pr_err("mcu_probe: mcu_dev register failed\n");
    goto error_mcu_dev;
  }

#ifdef CONFIG_HAS_EARLYSUSPEND
  /* To set BLANK_SCREEN level that prevent wrong-touch while talking */
  mcu_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
  mcu_data.early_suspend.suspend = mcu_early_suspend;
  mcu_data.early_suspend.resume = mcu_early_resume;
  register_early_suspend(&mcu_data.early_suspend);
#endif

#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
  mcu_reg = true;
#endif

  pr_debug("[MCU] %s -- leaving\n", __FUNCTION__);

  return 0;

error_mcu_dev:
  free_irq(client->irq, &mcu_data);
  pr_err("[MCU] probe error\n");
  return err;
}

static int mcu_remove(struct i2c_client *client)
{
  struct mcu_data *tp = i2c_get_clientdata(client);
  input_unregister_device(tp->input);
  free_irq(client->irq, tp);
#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&tp->early_suspend);
#endif

  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mcu_early_suspend(struct early_suspend *h)
{
  struct i2c_client *client = mcu_data.client;
  pr_debug("[MCU] %s ++ entering\n", __FUNCTION__);

#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
  led_off();
  screen_off_flag = true;
#endif

  check_led=1;
  if (led_flag==1 || led_flag==3)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_CLOW_BAT);
  else if (led_flag==2)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_MISSCALL);

  pr_debug("[MCU] %s -- leaving\n", __FUNCTION__);
}

static void mcu_early_resume(struct early_suspend *h)
{
  pr_debug("[MCU] %s ++ entering\n", __FUNCTION__);
  check_led=0;
  fake_resume=0;
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
  cancel_delayed_work(&led_wq);
  screen_off_flag = false;
  led_on();
  schedule_delayed_work(&led_wq, msecs_to_jiffies(KPD_LED_DELAY_TIME));
#endif

  batt_info_update();
  mcu_power_mode(MCU_SYSTEM_NORMAL);

  pr_debug("[MCU] %s -- leaving\n", __FUNCTION__);
}
#endif

static int mcu_suspend(struct i2c_client *client, pm_message_t mesg)
{
  pr_debug("[MCU] low power suspend init done.\n");

  /* TODO: Prevent uP not release key to hang the interrupt */
  mcu_power_mode(MCU_SYSTEM_SLEEP);

  check_led=1;
  if (fake_resume==0) {
      if (led_flag==1 || led_flag==3)
          mcu_write(client,MCU_LED_STATUS,MCU_LED_CLOW_BAT);
      else if (led_flag==2)
          mcu_write(client,MCU_LED_STATUS,MCU_LED_MISSCALL);
  }
  return 0;
}

static int mcu_resume(struct i2c_client *client)
{
  pr_debug("[MCU] normal resume init done.\n");
  check_led=0;
  fake_resume=1;
  return 0;
}

static void mcu_work_func(struct work_struct *work)
{
  struct i2c_client *client = mcu_data.client;

  bool fuel_flush = false;
  uint8_t data_buf[2] = {0};
  uint8_t power_buf[2] = {0};
  uint8_t key_buf[2] = {0};
  uint8_t battery_buf[10] = {0};
  int16_t local_temp = 0x8000;
  uint8_t count_temp = 0;

  mutex_lock(&mcu_mutex);

  data_buf[0] = MCU_INTERRUPT;
  data_buf[1] = 0;
  if ( i2c_read(client, data_buf, 1) ) {
       mutex_unlock(&mcu_mutex);
       return;
  }
  pr_info("[MCU] 0x23 = %x\n", data_buf[0]);

  if(data_buf[0]&MCU_BATLOSS_EVENT) {
     mcu_write(client,0x7B,0xAA);
     pr_err("[MCU] Reset MCU and delay 1 sec\n");
     mdelay(1000);
     data_buf[0] = MCU_INTERRUPT;
     data_buf[1] = 0;
     if(i2c_read(client, data_buf, 1)) {
        mutex_unlock(&mcu_mutex);
        return;
     }
     if(data_buf[0]&MCU_BATLOSS_EVENT) {
        pr_err("[MCU] MCU_BATLOSS_EVENT\n");
        buf_data->have_battery = 0;
     }

     fuel_update(*buf_data);
  }

  if(data_buf[0]&MCU_POWER_EVENT) {
     fuel_flush = true;
     power_buf[0] = MCU_POWER_STATUS;
     power_buf[1] = 0;
     if ( i2c_read(client, power_buf, 1) ) {
          mutex_unlock(&mcu_mutex);
          return;
     }
     pr_info("[MCU] 0x24 = %x\n", power_buf[0]);
     if(power_buf[0]&MCU_EXT_POWER_OUT)
     {
        buf_data->charger_type = ACER_CHARGER_TYPE_NO_CHARGER;
        pr_info("[MCU] CABLE-OUT\n");
     }else{
        if(power_buf[0]&MCU_AC_CABLE)
        {
            buf_data->charger_type = ACER_CHARGER_TYPE_IS_AC;
            pr_debug("[MCU] AC type\n");
        }else{
            mcu_write(mcu_data.client,MCU_USB_CHARGE,MCU_USB_CHARGE_500MA);
            buf_data->charger_type = ACER_CHARGER_TYPE_IS_USB;
            pr_info("[MCU] USB type\n");
        }
     }
     if(power_buf[0]&MCU_NO_CHARGING)
     {
        buf_data->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
     }
     else{
        buf_data->charge_status = POWER_SUPPLY_STATUS_CHARGING;
     }

     data_buf[0] |= MCU_BATCAP_EVENT;
  }

  if(data_buf[0]&MCU_BATCAP_EVENT || data_buf[0]&MCU_TEMP_EVENT) {
     fuel_flush = true;
     battery_buf[0] = MCU_BATTERY_SOC;
     battery_buf[1] = 0;
     if( i2c_read(client, battery_buf, 10) ) {
         mutex_unlock(&mcu_mutex);
         return;
     }

     if( abs(battery_buf[0] - last_level) > 3 && Count < 3) {
         Count++;
         pr_err("[MCU] Count=%d,Wrong capacity (old,new) = (%d,%d)\n",Count,last_level,battery_buf[0]);
     } else {
         Count = 0;
         last_level = battery_buf[0];
         buf_data->capacity = battery_buf[0];
         pr_info("[MCU] capacity = %d\n",buf_data->capacity);
     }

     // temperature range: -255.0C ~ 255.0C
     local_temp = ((int16_t)battery_buf[3]) * ((battery_buf[4]&0x80)?-10:10);
     // check temperature value
     count_temp = 0;
     while( (battery_buf[4]&0x7F || local_temp > 700 || local_temp < -300 ||
             abs(local_temp - last_temp) > 600) && count_temp++ < 2 ) {
         // retry if (1)>70.0C or (2)<-30.0C or (3)change >60.0C
         pr_err("[MCU] Wrong temperature: %d,%d... read again\n", battery_buf[4], battery_buf[3]);
         pr_err("[MCU] Count=%d, (old,new) = (%d,%d)\n",count_temp,last_temp,local_temp);
         battery_buf[0] = MCU_BATTERY_SOC;
         battery_buf[1] = 0;
         if( i2c_read(client, battery_buf, 10) ) {
             mutex_unlock(&mcu_mutex);
             return;
         }
         local_temp = ((int16_t)battery_buf[3]) * ((battery_buf[4]&0x80)?-10:10);
     }
     buf_data->temperature = local_temp;
     last_temp = local_temp;
     if( (buf_data->temperature >= MAX_TEMPERATURE || battery_buf[4]&0x80)
             && buf_data->charger_type != ACER_CHARGER_TYPE_NO_CHARGER ) {
         // battery overheat if temperature >45.0C or <0.0C
         buf_data->health = POWER_SUPPLY_HEALTH_OVERHEAT;
         pr_err("[MCU] Over Temprature: %d\n", buf_data->temperature/10);
     }
     else
         buf_data->health = POWER_SUPPLY_HEALTH_GOOD;
     pr_info("[MCU] %d,%d: temperature = %d\n",battery_buf[4],battery_buf[3],buf_data->temperature/10);

     buf_data->voltage = (((uint16_t)battery_buf[6])<<8)|battery_buf[5];
     pr_debug("[MCU] voltage = %d\n",buf_data->voltage);
  }

  if(fuel_flush) {
     fuel_flush = false;
     fuel_update(*buf_data);
  }

  if(data_buf[0]&MCU_BATLOW_EVENT) {
     pr_debug("[MCU] MCU_BATLOW_EVENT\n");
  }

  if(data_buf[0]&MCU_BATVLOW_EVENT) {
     pr_info("[MCU] MCU_BATVLOW_EVENT ,charger = %d\n",buf_data->charger_type);
     if( buf_data->charger_type == ACER_CHARGER_TYPE_NO_CHARGER ) {
	     wake_lock_timeout(&work_wake_lock, HZ * 10);
	     buf_data->capacity = 0;
	     fuel_update(*buf_data);
     }
  }

  if(data_buf[0]&MCU_HDQ_EVENT) {
     pr_debug("[MCU] MCU_HDQ_EVENT\n");
  }

  if(data_buf[0]&MCU_QWKEY_EVENT) {
     key_buf[0] = MCU_KEYBOARD_DATA;
     key_buf[1] = 0;
     if ( i2c_read(client, key_buf, 2) ) {
          mutex_unlock(&mcu_mutex);
          return;
     }
     pr_debug("[MCU] 0x30 = %x\n", key_buf[0]);
     pr_debug("[MCU] 0x31 = %x\n", key_buf[1]);
#if defined (CONFIG_ACER_A3_KEYPAD)
     a3_keypad_report_key(1, key_buf);
#endif
  }

   mutex_unlock(&mcu_mutex);
}

static irqreturn_t mcu_interrupt(int irq, void *dev_id)
{
  /* TODO: Remove mdelay() to prevent listening */
  /*       music delay on BT Headset via A2DP   */
  disable_irq(irq);
  schedule_work(&mcu_data.work);
  enable_irq(irq);
  return IRQ_HANDLED;
}

/* open command for MCU device file	*/
static int mcu_open(struct inode *inode, struct file *file)
{
  return 0;
}
/* close command for MCU device file */
static int mcu_close(struct inode *inode, struct file *file)
{
  return 0;
}

static int mcu_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  int err = 0;
  struct i2c_client *client = mcu_data.client;
  uint8_t data_buf[2] = {0};

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
  if( client == NULL){
    pr_err("I2C driver not install (MCU_ioctl)\n");
    return -EFAULT;
  }

  /* cmd mapping */
  switch(cmd){
  case IOCTL_SET_RED_LED:
    if(arg)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_RED);
    else
       mcu_write(client,MCU_LED_STATUS,MCU_LED_OFF);
   break;
  case IOCTL_SET_GREEN_LED:
    if(arg)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_GERRN);
    else
       mcu_write(client,MCU_LED_STATUS,MCU_LED_OFF);
    break;
  case IOCTL_SET_WHITE_LED:
    if(arg)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_WHITE);
    else
       mcu_write(client,MCU_LED_STATUS,MCU_LED_OFF);
    break;
  case IOCTL_SET_KEY_LED:
    if(arg)
       mcu_write(client,MCU_PWM_PERIOD,MCU_LED_TPKEY_ON);
    else
       mcu_write(client,MCU_PWM_PERIOD,MCU_LED_TPKEY_OFF);
    break;
  case IOCTL_GET_AVE_CUTTENT:

       data_buf[0] = MCU_BATTERY_AVE_CURRENT;
       data_buf[1] = 0;
       if ( i2c_read(client, data_buf, 2) ) {
            return -1;
       }
       err = (data_buf[0] + (data_buf[1]<<8));
       pr_info("[MCU] 0x2C = %x %x\n", data_buf[0],data_buf[1]);
       pr_info("[MCU] Average Current = %d\n", err);

    return err;
  case IOCTL_SET_MISSING_CALL_LED:
    if (arg) {
        led_flag |= 0x2;
        if (check_led==1 || fake_resume==1)
            mcu_write(client,MCU_LED_STATUS,MCU_LED_MISSCALL);
    } else
        led_flag &= ~(0x2);
    break;
  case IOCTL_SET_LOW_BATTERY_LED:
    if(arg)
       mcu_write(client,MCU_LED_STATUS,MCU_LED_LOW_BAT);
    else
       mcu_write(client,MCU_LED_STATUS,MCU_LED_OFF);
    break;
  case IOCTL_SET_CRITICAL_LOW_LED:
    if(arg) {
        mcu_write(client,MCU_LED_STATUS,MCU_LED_CLOW_BAT);
        mdelay(500);
        mcu_write(client,MCU_LED_STATUS,MCU_LED_OFF);
        led_flag |= 0x1;
    } else
        led_flag &= ~(0x1);
    break;
  case IOCTL_KPD_LED_ON:
    mcu_write(mcu_data.client,MCU_LED_STATUS,MCU_LED_WHITE);
    mdelay(10);
    mcu_write(mcu_data.client,MCU_PWM_PERIOD,MCU_LED_TPKEY_ON);
    break;
  case IOCTL_KPD_LED_OFF:
    if (check_led==1)
        break;
    mcu_write(mcu_data.client,MCU_LED_STATUS,MCU_LED_OFF);
    mdelay(10);
    mcu_write(mcu_data.client,MCU_PWM_PERIOD,MCU_LED_TPKEY_OFF);
    break;
  case IOCTL_GET_MCU_VERSION:
       data_buf[0] = MCU_SYSTEM_VERSION;
       data_buf[1] = 0;
       if ( i2c_read(client, data_buf, 1) ) {
            return -1;
       }
       err = data_buf[0];
       pr_info("[MCU] Version = %d\n", err);
    return err;
  case IOCTL_SET_CHARGE_CONTROL:
    if(arg)
       mcu_write(client,MCU_CHARGE_CONTROL,MCU_CHARGE_ON);
    else
       mcu_write(client,MCU_CHARGE_CONTROL,MCU_CHARGE_OFF);
    break;
  case IOCTL_MCU_WRITE_DATA:
    data_buf[0] = arg;
    data_buf[1] = arg>>8;
    mcu_write(client,data_buf[1],data_buf[0]);
    pr_info("[MCU] Write address= 0x%X, cmd= 0x%X\n",data_buf[1],data_buf[0]);
    break;
  case IOCTL_MCU_READ_DATA:
    data_buf[0] = arg;
    data_buf[1] = 0;
    if ( i2c_read(client, data_buf, 1) ) {
        return -1;
    }
    err = data_buf[0];
    pr_info("[MCU] Read address= 0x%X, data= 0x%X\n", (int)arg,err);
    break;
  default:
    return -1;
  }

  return 0;
}

static void __exit mcu_exit(void)
{
  i2c_del_driver(&mcu_driver);
}

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
  int rc;
  struct i2c_msg msgs[] = {
    {
        .addr = client->addr,
        .flags = 0,
        .len = 1,
        .buf = buf,
    },
    {
        .addr = client->addr,
        .flags = I2C_M_RD,
        .len = count,
        .buf = buf,
    },
  };

  rc = i2c_transfer(client->adapter, msgs, 2);
  if (rc < 0) {
      pr_err("[MCU] %s error %d,read addr= 0x%x,len= %d\n", __FUNCTION__,rc,*buf,count);
      mdelay(200);
      rc = mcu_write(client,0x7B,0xAA);
      if(rc == 0) {
        pr_err("[MCU] reset success\n");
        mdelay(1000);
      }
      return -1;
  }
  return 0;
}

/*
 * client: target client
 * buf: target register with command
 */
static int i2c_write(struct i2c_client *client, char *buf)
{
  /* buf[0] -> target reg. info. */
  /* buf[1] -> cmd */
  if(2 != i2c_master_send(client, buf, 2)){
    pr_err("[MCU] %s addr 0x%x data 0x%x error\n",__FUNCTION__,buf[0],buf[1]);
    return -1;
  }
  return 0;
}

static int mcu_write(struct i2c_client *client,char reg, char cmd)
{
  int rc;
  uint8_t data_buf[2] = {0};

  data_buf[0] = reg;
  data_buf[1] = cmd;
  rc = i2c_write(client, data_buf);
  if(rc < 0)
     return -1;
  return 0;
}

void mcu_dark_mode(void)
{
  mcu_write(mcu_data.client,MCU_LED_STATUS,MCU_LED_OFF);
  mdelay(10);
  mcu_write(mcu_data.client,MCU_PWM_PERIOD,MCU_LED_TPKEY_OFF);
  mdelay(10);
  mcu_power_mode(MCU_SYSTEM_DARK);
}
EXPORT_SYMBOL(mcu_dark_mode);

#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
static void led_on(void)
{
  mcu_write(mcu_data.client,MCU_LED_STATUS,MCU_LED_WHITE);
  mdelay(10);
  mcu_write(mcu_data.client,MCU_PWM_PERIOD,MCU_LED_TPKEY_ON);
}

static void led_off(void)
{
  if (check_led==0) {
      mcu_write(mcu_data.client,MCU_LED_STATUS,MCU_LED_OFF);
      mdelay(10);
      mcu_write(mcu_data.client,MCU_PWM_PERIOD,MCU_LED_TPKEY_OFF);
  }
}
#endif

#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
void kpd_led_on(void)
{
  cancel_delayed_work(&led_wq);
  led_on();
}
EXPORT_SYMBOL(kpd_led_on);

void kpd_led_off(void)
{
  schedule_delayed_work(&led_wq, msecs_to_jiffies(KPD_LED_DELAY_TIME));
}
EXPORT_SYMBOL(kpd_led_off);

static void mcu_led_work_func(struct work_struct *work)
{
  led_off();
  pr_debug("[MCU] Enter LED delay 10 Sec\n");
}

int mcu_probe_check(void)
{
  return mcu_reg;
}
EXPORT_SYMBOL(mcu_probe_check);

int screen_off_check(void)
{
  return screen_off_flag;
}
EXPORT_SYMBOL(screen_off_check);
#endif

static void batt_info_update(void)
{
  uint8_t info_buf[2] = {0};
  info_buf[0] = MCU_BATTERY_SOC;
  i2c_read(mcu_data.client, info_buf, 1);
  buf_data->capacity = info_buf[0];
  pr_info("[MCU] mcu_early_resume capacity (new,old) = (%d,%d)\n",info_buf[0],last_level);
  last_level = info_buf[0];
  fuel_update(*buf_data);
}

static void mcu_power_mode(char mode)
{
  mcu_write(mcu_data.client,MCU_SYSTEM_STATUS,mode);
  pr_debug("[MCU] power mode %x\n",mode);
}

module_init(mcu_init);
module_exit(mcu_exit);

MODULE_AUTHOR("Jimbo Lo<Jimbo_lo@acer.com.tw>");
MODULE_DESCRIPTION("MCU micro-P driver");
