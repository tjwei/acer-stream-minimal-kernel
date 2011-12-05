/* First release: 2009.08.04
 * ADI ADV752X HDMI IC
 */

#include <linux/ioctl.h>
#include <linux/platform_device.h>
/*
 * Return Values from Interrupt Handler
 */

#define USE_AV_MUTE            0
#define HPD_INT                0x80
#define RX_SENSE_INT           0x40
#define VSYNC_INT              0x20
#define ADUIO_FIFO_INT         0x10
#define EDID_INT               0x04
#define INT_ERROR              0x80

#define MDP_LCDC_PCLK_CLK      42
#define LCDC_BASE              0xE0000

/* Timer define */
#define AV_MUTE_TIME           0000  /* 1s */
#define READ_EDID_TIME         500
#define POLLING_EDID           2000
#define DETACH_TIME            1000

/*  CEC timing define */
#define CEC_INITIALIZE_2D_48 0x0C,0xFB,0x0C,0x1E,0x0D,0xD9,0x0A,0xAC,0x09,0xCF,0x0B,   \
                             0x8A,0x06,0xEC,0x05,0xA0,0x08,0x39,0x01,0xBB,0x04,0x54,   \
                             0x05,0x31,0x03,0x07,0x0A,0x62
#define CEC_INITIALIZE_4A_4B 0x00,0xB9
#define CEC_INITIALIZE_4D_52 0x00,0xDE,0x02,0x99,0x03,0x76

/* MAIN define  */
#define MAIN_STATUS                            0x42
#define MAIN_INTERRUPT_BLOCK1                  0x96
#define MAIN_INTERRUPT_BLOCK2                  0x97

/* CEC define */
#define CEC_TX_FRAME_HEADER                    0x00
#define CEC_TX_FRAME_LENGTH                    0x10
#define CEC_TX_ENABLE                          0x11
#define CEC_TX_NACK_COUNTER                    0x14
#define CEC_RX_ENABLE                          0x26
#define CEC_RX_LOGICAL_ADDRESS_MASK            0x27
#define CEC_RX_LOGICAL_ADDRESS0                0x28
#define CEC_CLOCK_DIVIDER_POWER_MODE           0x2A
#define CEC_LA_CANDIDATE                       4,8,11

/* CEC opcode */
#define CEC_OPCODE_GIVE_DECK_STATUS            0x1A
#define CEC_OPCODE_PLAY                        0x41
#define CEC_OPCODE_DECK_CONTROL                0x42
#define CEC_OPCODE_USER_CONTROL_PRESSED        0x44
#define CEC_OPCODE_USER_CONTROL_RELEASED       0x45
#define CEC_OPCODE_GIVE_OSD_NAME               0x46
#define CEC_OPCODE_SET_OSD_NAME                0x47
#define CEC_OPCODE_GIVE_PHYSICAL_ADDRESS       0x83
#define CEC_OPCODE_REQUEST_ACTIVE_SOURCE       0x85
#define CEC_OPCODE_SET_STREAM_PATH             0x86
#define CEC_OPCODE_GIVE_DRVICE_VENDOR_ID       0x8C
#define CEC_OPCODE_MENU_REQUEST                0x8D
#define CEC_OPCODE_GIVE_DRVICE_POWER_STATUS    0x8F
#define CEC_OPCODE_CEC_VERSION                 0x9F


/* CEC opcode send */
#define CEC_OPCODE_REPORT_PHYSICAL_ADDRESS     0x84


/*  CEC  UI command*/
enum {
    CEC_UICMD_SELECT        = 0x0,
    CEC_UICMD_UP,
    CEC_UICMD_DOWN,
    CEC_UICMD_LEFT,
    CEC_UICMD_RIGHT,
    CEC_UICMD_MENU          = 0xA,
    CEC_UICMD_BACK          = 0xD,
    CEC_UICMD_PLAY          = 0x44,
    CEC_UICMD_STOP          = 0x45,
    CEC_UICMD_PAUSE         = 0x46,
    CEC_UICMD_REWIND        = 0x48,
    CEC_UICMD_FAST_FORWARD,
    CEC_UICMD_FORWARD       = 0x4B,
    CEC_UICMD_BACKWARD,
};

enum mytiming {
    HTX_480i,
    HTX_576i,
    HTX_720_480p,
    HTX_720_576p,
    HTX_720p_60,
    HTX_720p_50,
    HTX_1080i_30,
    HTX_1080i_25,
    HTX_640_480p,
    LCD_480_800
};

typedef enum mytiming Timing;

enum myaspectratio {
    _4x3,
    _16x9
};

typedef enum myaspectratio AspectRatio;

enum videocodes {
    F_640_480p_60    = 1, /* F : Four by three    S : Sixteen by nine */
    F_720_480p_60,
    S_720_480p_60,
    S_1280_720p_60,
    S_1280_720p_50   = 19
};


struct edid_data {
    int support_format;
    bool vsdb_hdmi;
    uint8_t hdmi_addr_ab;
    uint8_t hdmi_addr_cd;
    uint8_t segment_num;
    uint8_t segment_current;
};


/*
 *    Declarations of functions
 */
int i2c_read(struct i2c_client *client,uint8_t addr , uint8_t *buf , int length);
int i2c_write(struct i2c_client *client, char *buf, int count);
int i2c_write_byte(struct i2c_client *client, uint8_t addr, uint8_t buf);
int i2c_write_bit(struct i2c_client *client, uint8_t addr,uint8_t bit, bool en);

void turn_on_HDMI(void);
int turn_off_HDMI(void);

void add_cea( uint8_t *edid_dat);
int16_t get_bit( int16_t in, int16_t high_bit, int16_t low_bit );

/* cec*/
void analyse_msg(uint8_t *msg);
void init_cec(struct i2c_client *main,struct i2c_client *cec,struct edid_data ed);

void set_hdmi_platform_device(struct platform_device *platform);
/*  IO CTL */
int adi752x_open(struct inode *inode, struct file *file);
int adi752x_close(struct inode *inode, struct file *file);
int adi752x_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

#define IOC_HDMI_ADI752X_MAGIC                0x66
#define IOC_MAXNR                             10
#define IOCTL_HDMI_OS_RESOLUTION              _IOR(IOC_HDMI_ADI752X_MAGIC, 1, int)
#define IOCTL_HDMI_GET_TV_FORMATS             _IOR(IOC_HDMI_ADI752X_MAGIC, 2, int)
#define IOCTL_HDMI_SET_TV_FORMATS             _IOW(IOC_HDMI_ADI752X_MAGIC, 3, int)
#define IOCTL_HDMI_START                      _IOR(IOC_HDMI_ADI752X_MAGIC, 4, int)
#define IOCTL_HDMI_END                        _IOR(IOC_HDMI_ADI752X_MAGIC, 5, int)
#define IOCTL_HDMI_GET_MODE                   _IOR(IOC_HDMI_ADI752X_MAGIC, 6, int)
#define IOCTL_HDMI_SET_MODE                   _IOW(IOC_HDMI_ADI752X_MAGIC, 7, int)
#define IOCTL_HDMI_GET_CURRENT_FORMAT         _IOR(IOC_HDMI_ADI752X_MAGIC, 8, int)


