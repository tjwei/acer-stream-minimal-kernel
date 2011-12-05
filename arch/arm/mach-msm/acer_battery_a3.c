// 2009.10.29 first release

#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <mach/mcu.h>

#define BATTERY_DRIVER_NAME             "acer-battery"
#define UNKNOWN                         0

struct bq27010_data {
	struct fuel_data data;
	struct power_supply ac;
	struct power_supply usb;
	struct power_supply battery;
};

static struct bq27010_data *fuel_info;

void fuel_update(struct fuel_data buf_data)
{
	fuel_info->data = buf_data;

	power_supply_changed(&fuel_info->ac);
	power_supply_changed(&fuel_info->usb);
	power_supply_changed(&fuel_info->battery);
}

EXPORT_SYMBOL(fuel_update);

static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (ACER_CHARGER_TYPE_IS_AC == fuel_info->data.charger_type)?1:0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (ACER_CHARGER_TYPE_IS_USB == fuel_info->data.charger_type)?1:0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	if(!fuel_info->data.have_battery) {
		val->intval = UNKNOWN;
		return 0;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if(fuel_info->data.capacity == 100 &&
			fuel_info->data.charger_type != ACER_CHARGER_TYPE_NO_CHARGER)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = fuel_info->data.charge_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = fuel_info->data.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fuel_info->data.voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = fuel_info->data.capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = fuel_info->data.temperature;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int bq27010_probe(struct platform_device *pdev)
{
	int ret = 0;
	pr_info("[BATT] Enter probe\n");
	fuel_info = kzalloc(sizeof(struct bq27010_data), GFP_KERNEL);
	if (fuel_info == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

	fuel_info->ac.properties = ac_props;
	fuel_info->ac.num_properties = ARRAY_SIZE(ac_props);
	fuel_info->ac.get_property = ac_get_property;
	fuel_info->ac.name = "ac";
	fuel_info->ac.type = POWER_SUPPLY_TYPE_MAINS;

	fuel_info->usb.properties = usb_props;
	fuel_info->usb.num_properties = ARRAY_SIZE(usb_props);
	fuel_info->usb.get_property = usb_get_property;
	fuel_info->usb.name = "usb";
	fuel_info->usb.type = POWER_SUPPLY_TYPE_USB;

	fuel_info->battery.properties = battery_props;
	fuel_info->battery.num_properties = ARRAY_SIZE(battery_props);
	fuel_info->battery.get_property = battery_get_property;
	fuel_info->battery.name = "battery";
	fuel_info->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	ret = power_supply_register(&pdev->dev, &fuel_info->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &fuel_info->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &fuel_info->battery);
	if (ret)
		goto err_battery_failed;

	platform_set_drvdata(pdev, fuel_info);

	pr_info("[BATT] probe done\n");
	return 0;

err_battery_failed:
	power_supply_unregister(&fuel_info->battery);
err_usb_failed:
	power_supply_unregister(&fuel_info->usb);
err_ac_failed:
	power_supply_unregister(&fuel_info->ac);
err_data_alloc_failed:
	pr_err("[BATT] probe error\n");
	return ret;
}

static int bq27010_remove(struct platform_device *pdev)
{
	struct bq27010_data *fuel_info = platform_get_drvdata(pdev);

	power_supply_unregister(&fuel_info->ac);
	power_supply_unregister(&fuel_info->usb);
	power_supply_unregister(&fuel_info->battery);

	kfree(fuel_info);
	fuel_info = NULL;
	return 0;
}

static struct platform_driver bq27010_driver = {
	.probe		= bq27010_probe,
	.remove		= bq27010_remove,
	.driver = {
		.name = BATTERY_DRIVER_NAME
	}
};

static int __init bq27010_init(void)
{
	return platform_driver_register(&bq27010_driver);
}

static void __exit bq27010_exit(void)
{
	platform_driver_unregister(&bq27010_driver);
}

module_init(bq27010_init);
module_exit(bq27010_exit);

MODULE_LICENSE("GPL v1");
