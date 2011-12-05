#if defined(CONFIG_ACER_DEBUG)
#define DEBUG
#endif
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/board.h>
#include "tsl2563.h"


/*
*
*	register definitions
*
*/
#define TSL2563_CONTROL_REG             0x80
#define TSL2563_TIMING_REG              0x81
#define TSL2563_THRES_LOW_LSB_REG       0x82
#define TSL2563_THRES_LOW_MSB_REG       0x83
#define TSL2563_THRES_HIGH_LSB_REG      0x84
#define TSL2563_THRES_HIGH_MSB_REG      0x85
#define TSL2563_INTERRUPT_REG           0x86
#define TSL2563_CRC_REG                 0x88
#define TSL2563_ID_REG                  0x8A
#define TSL2563_DATA0_LSB_REG           0x8C
#define TSL2563_DATA0_MSB_REG           0x8D
#define TSL2563_DATA1_LSB_REG           0x8E
#define TSL2563_DATA1_MSB_REG           0x8F

#define TSL2563_GET_BITSLICE(regvar, bitname)\
	(regvar & bitname##__MSK) >> bitname##__POS


#define TSL2563_SET_BITSLICE(regvar, bitname, val)\
	(regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

/*
*
*	bit slice positions in registers
*
*/

#define TSL2563_POWER__POS      0
#define TSL2563_POWER__LEN      2
#define TSL2563_POWER__MSK      0x03
#define TSL2563_POWER__REG      TSL2563_CONTROL_REG

#define TSL2563_GAIN__POS       4
#define TSL2563_GAIN__LEN       1
#define TSL2563_GAIN__MSK       0x10
#define TSL2563_GAIN__REG       TSL2563_TIMING_REG

#define TSL2563_INTEG__POS      3
#define TSL2563_INTEG__LEN      1
#define TSL2563_INTEG__MSK      0x08
#define TSL2563_INTEG__REG      TSL2563_TIMING_REG

#define TSL2563_INTEG_TIME__POS  0
#define TSL2563_INTEG_TIME__LEN  2
#define TSL2563_INTEG_TIME__MSK  0x03
#define TSL2563_INTEG_TIME__REG  TSL2563_TIMING_REG

#define TSL2563_THRES_LOW_LSB__POS     0
#define TSL2563_THRES_LOW_LSB__LEN     8
#define TSL2563_THRES_LOW_LSB__MSK     0xFF
#define TSL2563_THRES_LOW_LSB__REG     TSL2563_THRES_LOW_LSB_REG

#define TSL2563_THRES_LOW_MSB__POS     0
#define TSL2563_THRES_LOW_MSB__LEN     8
#define TSL2563_THRES_LOW_MSB__MSK     0xFF
#define ISL2563_THRES_LOW_MSB__REG     TSL2563_THRES_LOW_MSB_REG

#define TSL2563_THRES_HIGH_LSB__POS     0
#define TSL2563_THRES_HIGH_LSB__LEN     8
#define TSL2563_THRES_HIGH_LSB__MSK     0xFF
#define TSL2563_THRES_HIGH_LSB__REG     TSL2563_THRES_HIGH_LSB_REG

#define TSL2563_THRES_HIGH_MSB__POS     0
#define TSL2563_THRES_HIGH_MSB__LEN     8
#define TSL2563_THRES_HIGH_MSB__MSK     0xFF
#define ISL2563_THRES_HIGH_MSB__REG     TSL2563_THRES_HIGH_MSB_REG

#define TSL2563_DATA0_LSB__POS      0
#define TSL2563_DATA0_LSB__LEN      8
#define TSL2563_DATA0_LSB__MSK      0xFF
#define TSL2563_DATA0_LSB__REG      TSL2563_DATA0_LSB_REG

#define TSL2563_DATA0_MSB__POS      0
#define TSL2563_DATA0_MSB__LEN      8
#define TSL2563_DATA0_MSB__MSK      0xFF
#define TSL2563_DATA0_MSB__REG      TSL2563_DATA0_MSB_REG

#define TSL2563_DATA1_LSB__POS      0
#define TSL2563_DATA1_LSB__LEN      8
#define TSL2563_DATA1_LSB__MSK      0xFF
#define TSL2563_DATA1_LSB__REG      TSL2563_DATA1_LSB_REG

#define TSL2563_DATA1_MSB__POS      0
#define TSL2563_DATA1_MSB__LEN      8
#define TSL2563_DATA1_MSB__MSK      0xFF
#define TSL2563_DATA1_MSB__REG      TSL2563_DATA1_MSB_REG

#define _I2C_READ_BIT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##__REG;\
	i2c_read(client, reg_buf, 1);\
	czdata[0] = TSL2563_GET_BITSLICE(reg_buf[0], bitname);\
}

#define _I2C_WRITE_BIT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##__REG;\
	i2c_read(client, reg_buf, 1);\
	reg_buf[1] = TSL2563_SET_BITSLICE(reg_buf[0], bitname, czdata);\
	reg_buf[0] = bitname##__REG;\
	i2c_write(client, reg_buf, 2);\
}

#define _I2C_READ_SHORT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##_LSB__REG;\
	i2c_read(client, reg_buf, 2);\
	memcpy(czdata, reg_buf, 2);\
}

#define _I2C_WRITE_SHORT(bitname, czdata)\
{\
	unsigned char reg_buf[3] = {0};\
	reg_buf[0] = bitname##_LSB__REG;\
	memcpy(&(reg_buf[1]), czdata, 2);\
	i2c_write(client, reg_buf, 3);\
}

static int __init tsl_init(void);
static int tsl_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tsl_remove(struct i2c_client *client);
static int tsl_suspend(struct i2c_client *client, pm_message_t mesg);
static int tsl_resume(struct i2c_client *client);
static int tsl_open(struct inode *inode, struct file *file);
static int tsl_close(struct inode *inode, struct file *file);
static int tsl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static void __exit tsl_exit(void);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);


static const struct i2c_device_id tsl_id[] = {
	{ TSL2563_DRIVER_NAME, 0 },
	{ }
};

/* Data for I2C driver */
struct tsl_data {
	struct i2c_client *client;
	struct work_struct work;
	wait_queue_head_t wait;
};
static struct tsl_data *tsl_data;

/*File operation of ISL device file */
static const struct file_operations tsl_fops = {
	.owner		= THIS_MODULE,
	.open		= tsl_open,
	.release	= tsl_close,
	.ioctl		= tsl_ioctl,
};

/* new style I2C driver struct */
static struct i2c_driver tsl_driver = {
	.probe = tsl_probe,
	.remove = __devexit_p(tsl_remove),
	.id_table = tsl_id,
	.suspend = tsl_suspend,
	.resume = tsl_resume,
	.driver = {
		.name = TSL2563_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static struct miscdevice tsl_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TSL2563_DRIVER_NAME,
	.fops = &tsl_fops,
};

static int __init tsl_init(void)
{
	int res;
	if(hw_version == 1){
		pr_err("HW_VER = %d, [TSL] TSL2563 will init\n",hw_version);
		pr_info("[TSL] TSL2563 init\n");
		res = i2c_add_driver(&tsl_driver);
		if (res) {
			pr_err("[TSL] %s: Driver Initialisation failed\n", __FILE__);
		}
	}else{
		pr_err("HW_VER = %d, [TSL] TSL2563 won't init\n",hw_version);
		res = -1;
	}
	return res;
}

/* reset to initial value and confirm that is correct */
static int reset_and_check_value(struct i2c_client *client)
{
	unsigned char reg_buf[2] = {0x80,0x03};
	unsigned char reg_buf1[2] = {0x81,0x1A};
	//int i = 0;

	pr_debug("[TSL] %s ++ entering\n", __FUNCTION__);

	if(2 != i2c_master_send(client, reg_buf, 2)){
		pr_err("[TSL] i2c_read0 --> Send reg. info error\n");
		return -1;
	}
	if(2 != i2c_master_send(client, reg_buf1, 2)){
		pr_err("[TSL] i2c_read1 --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

static int tsl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;

	pr_debug("%s ++ entering\n", __FUNCTION__);
	tsl_data = kzalloc(sizeof(struct tsl_data),GFP_KERNEL);
	if (NULL==tsl_data) {
		res = -ENOMEM;
		goto out;
	}
	tsl_data->client = client;

	/* check i2c functionality is workable */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[ISL] i2c_check_functionality error!\n");
		res = -ENOTSUPP;
		goto out_free_mem;
	}
	strlcpy(client->name, TSL2563_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, tsl_data);
	/* check TSL2563 chip is workable */
	if (-1==reset_and_check_value(client)) {
		pr_err("[TSL] TSL2563 is not workable on i2c bus!\n");
		res = -ENXIO;
		goto out_free_mem;
	}

	/* register misc device */
	res = misc_register(&tsl_dev);
	if (res < 0) {
		pr_err("[TSL]: tsl_dev register failed! error code:[%d]\n", res);
		goto out_unreg_irq;
	}

	pr_info("[TSL] probe done\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;

out_unreg_irq:
	free_irq(client->irq, tsl_data);
out_free_mem:
	kfree(tsl_data);
out:
	pr_err("[TSL] probe error\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return res;
}

static int tsl_remove(struct i2c_client *client)
{
	misc_deregister(&tsl_dev);
	kfree(tsl_data);
	pr_info("[ISL] remove done\n");
	return 0;
}

static int tsl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	pr_debug("[BMA150] low power suspend init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static int tsl_resume(struct i2c_client *client)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	pr_debug("[BMA150] normal resume init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}



/*	open command for ISL device file	*/
static int tsl_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = tsl_data->client;
	if( client == NULL ){
		pr_err("[TSL] I2C driver not install (tsl_open)\n");
		return -1;
	}
	pr_debug("[TSL] has been opened\n");
	return 0;
}

static int tsl_close(struct inode *inode, struct file *file)
{
	pr_debug("[TSL] has been closed\n");
	return 0;
}

static int tsl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int tsl_cmd = 0;
	unsigned char data[2] = {0};
	struct i2c_client *client = tsl_data->client;

	/* check cmd */
	if (_IOC_TYPE(cmd) != TSL2563_IOC_MAGIC) {
		pr_err("[ISL] cmd magic type error\n");
		return -ENOTTY;
	}
	tsl_cmd = _IOC_NR(cmd);
	if (tsl_cmd >= TSL2563_IOC_MAXNR) {
		pr_err("[TSL] cmd number error\n");
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		pr_err("[TSL] cmd access_ok error\n");
		return -EFAULT;
	}
	if (client == NULL) {
		pr_err("[TSL] I2C driver not install (tsl_ioctl)\n");
		return -EFAULT;
	}

	/* cmd mapping */
	switch (cmd) {
		case TSL2563_SET_POWER:
			pr_debug("[TSL] TSL2563_SET_POWER\n");
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[TSL] TSL2563_SET_POWER: copy_from_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_BIT(TSL2563_POWER, data[0]);
			break;
		case TSL2563_SET_GAIN:
			pr_debug("[TSL] TSL2563_SET_GAIN\n");
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[TSL] TSL2563_SET_GAIN: copy_to_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_BIT(TSL2563_GAIN, data[0]);
			break;
		case TSL2563_SET_INTEG:
			pr_debug("[TSL] TSL2563_SET_INTEG\n");
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[TSL] TSL2563_SET_INTEG: copy_to_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_BIT(TSL2563_INTEG, data[0]);
			break;
		case TSL2563_SET_INTEG_TIME:
			pr_debug("[TSL] TSL2563_SET_INTEG_TIME\n");
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[TSL] TSL2563_SET_INTEG_TIME: copy_to_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_BIT(TSL2563_INTEG_TIME, data[0]);
			break;
		case TSL2563_SET_THRES_LOW:
			pr_debug("[TSL] TSL2563_SET_THRES_LOW\n");
			if (0!=copy_from_user((short*)data,(short*)arg,2)) {
				pr_err("[TSL] TSL2563_THRES_LOW: copy_from_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_SHORT(TSL2563_THRES_LOW, data);
			break;

		case TSL2563_SET_THRES_HIGH:
			pr_debug("[TSL] TSL2563_SET_THRES_HIGH\n");
			if (0!=copy_from_user((short*)data,(short*)arg,2)) {
				pr_err("[TSL] TSL2563_THRES_HIGH: copy_from_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_SHORT(TSL2563_THRES_HIGH, data);
			break;

		case TSL2563_GET_DATA0:
			pr_debug("[TSL] TSL2563_GET_DATA0\n");
			_I2C_READ_SHORT(TSL2563_DATA0, data);
			if (0!=copy_to_user((short*)arg,(short*)data,2)) {
				pr_err("[ISL] TSL2563_DATA0: copy_to_user error\n");
				return -EFAULT;
			}
			break;

		case TSL2563_GET_DATA1:
			pr_debug("[TSL] TSL2563_GET_DATA1\n");
			_I2C_READ_SHORT(TSL2563_DATA1, data);
			if (0!=copy_to_user((short*)arg,(short*)data,2)) {
				pr_err("[TSL] TSL2563_DATA1: copy_to_user error\n");
				return -EFAULT;
			}
			break;

		default:
			pr_err("[TSL] ioctl cmd not found\n");
			return -EFAULT;
	}
	return 0;
}

static void __exit tsl_exit(void)
{
	i2c_del_driver(&tsl_driver);
	pr_info("[TSL] TSL2563 exit\n");
}

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
	//Send target reg. info.
	if(1 != i2c_master_send(client, buf, 1)){
		pr_err("[TSL] i2c_read --> Send reg. info error\n");
		return -1;
	}
	//Get response data and set to buf
	if(count != i2c_master_recv(client, buf, count)){
		pr_err("[TSL] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

/*
 * client: target client
 * buf: target register with command
 * count: length of transmitting
 */
static int i2c_write(struct i2c_client *client, char *buf, int count)
{
	//buf[0] -> target reg. info.
	//buf[1] -> cmd1
	//buf[2] -> cmd2
	//buf...
	//printk(KERN_ERR "[ISL] Write to reg: 0x%02X, cmd=0x%02X. \n", buf[0],  buf[1]);
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[TSL] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

module_init(tsl_init);
module_exit(tsl_exit);

MODULE_AUTHOR("Andrew <Andrew_Chen@acer.com.tw>");
MODULE_DESCRIPTION("i2c TSL2563 driver");

