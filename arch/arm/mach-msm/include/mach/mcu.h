typedef enum
{
	ACER_CHARGER_TYPE_IS_AC = 0,
	ACER_CHARGER_TYPE_IS_USB,
	ACER_CHARGER_TYPE_NO_CHARGER,
	ACER_CHARGER_TYPE_INVALID =  0xFFFFFFFF,
} charger_type_t;

struct fuel_data
{
	bool have_battery;
	charger_type_t charger_type;
	__u8 charge_status;
	__u8 capacity;
	__u8 health;
	__u16 voltage;
	__s16 temperature;
};
/* Command Register list */
#define MCU_SYSTEM_VERSION        0x00
#define MCU_SYSTEM_STATUS         0x20
#define MCU_USB_CHARGE            0x22
#define MCU_INTERRUPT             0x23
#define MCU_POWER_STATUS          0x24
#define MCU_BATTERY_SOC           0x25
#define MCU_BATTERY_CAPACITY      0x26
#define MCU_BATTERY_TEMPERATURE   0x28
#define MCU_BATTERY_VOLTAGE       0x2A
#define MCU_BATTERY_AVE_CURRENT   0x2C
#define MCU_BATTERY_REAL_SOC      0x2E
#define MCU_HDQ_ADDRESS           0x2F
#define MCU_HDQ_DATA              0x30
#define MCU_KEYBOARD_DATA         0x31
#define MCU_PWM_PERIOD            0x33
#define MCU_LED_STATUS            0x34
#define MCU_CHARGE_CONTROL        0x7A
/* MCU Mode Register*/
#define MCU_SYSTEM_DARK           0x00
#define MCU_SYSTEM_SLEEP          0x01
#define MCU_SYSTEM_NORMAL         0x03
/* Charge status Register*/
#define MCU_USB_CHARGE_100MA      0x00
#define MCU_USB_CHARGE_500MA      0x01
/* Interrupt Event Register*/
#define MCU_POWER_EVENT           (1<<0)
#define MCU_BATCAP_EVENT          (1<<1)
#define MCU_BATLOW_EVENT          (1<<2)
#define MCU_BATVLOW_EVENT         (1<<3)
#define MCU_BATLOSS_EVENT         (1<<4)
#define MCU_HDQ_EVENT             (1<<5)
#define MCU_QWKEY_EVENT           (1<<6)
#define MCU_TEMP_EVENT            (1<<7)
/* LED Control Register*/
#define MCU_LED_OFF               0x00
#define MCU_LED_RED               (1<<0)
#define MCU_LED_GERRN             (1<<1)
#define MCU_LED_WHITE             (1<<2)
#define MCU_LED_MISSCALL          (1<<3)
#define MCU_LED_LOW_BAT           (1<<4)
#define MCU_LED_CLOW_BAT          (1<<5)
/* Touch Key LED Control Register */
#define MCU_LED_TPKEY_OFF         0x00
#define MCU_LED_TPKEY_ON          0x06
#define KPD_LED_DELAY_TIME        10000
/* Power status Register */
#define MCU_EXT_POWER_OUT        (1<<0)
#define MCU_NO_CHARGING          (1<<1)
#define MCU_AC_CABLE             (1<<2)
/* Charge Control Register */
#define MCU_CHARGE_OFF            0x55
#define MCU_CHARGE_ON             0x00
/* Warning Temperature */
#define MAX_TEMPERATURE           450
/* Extern Function */
extern void fuel_update(struct fuel_data buf_data);
extern void a3_keypad_report_key(int num, uint8_t buf[2]);
extern int a3_simple_test_report(void);
extern int a3_hdmi_nemo_ap_report(void);
extern void mcu_dark_mode(void);
#if defined (CONFIG_ACER_A3_KEYGUARD_SRS)
extern void kpd_led_on(void);
extern void kpd_led_off(void);
extern int mcu_probe_check(void);
extern int screen_off_check(void);
#endif
