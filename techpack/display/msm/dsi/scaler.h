#ifndef _SCALER_H_
#define _SCALER_H_


#define SCALER_I2C_NAME		"Realtek-scaler"
#define SCALER_I2C_ADDR		0x6E

#define DEVICE_CLASS_NAME 	"scaler"

#define READ_BUF_LEN			11

/*
#define STATUS_DISCONNECTED 	{ "SCALER_STATE=DISCONNECTED", NULL }
#define STATUS_CONNECTING 		{ "SCALER_STATE=CONNECTING", NULL }
#define STATUS_CONNECTED 		{ "SCALER_STATE=CONNECTED", NULL }
*/
#define SCALER_DEBUG_ON	        0

#define SCALER_INFO(fmt,arg...)    printk("<<-scaler-INFO->> "fmt"\n",##arg)
#define SCALER_ERROR(fmt,arg...)   printk("<<-scaler-ERROR->> "fmt"\n",##arg)
#define SCALER_DEBUG(fmt,arg...)   do {\
                                    if (SCALER_DEBUG_ON)\
                                        printk("<<-scaler-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                } while(0)

/* Registers */
#define PANEL_INIT_REG				0xA1		/* Notify scaler to init the panel */
#define INITIAL_STATUS_REG		0xA2		/* Nofity scaler the initial brightness and volume */
#define BATTERY_STATUS_REG		0xA3		/* Notify scaler the battery status, such as capacity, battery */
#define SHOW_OSD_REG				0xA4		/* Notify scaler to show the OSD */
#define GET_STATUS_REG			0xB1		/* Get the status from scaler */

enum	CONNECTION_STATUS {
	DISCONNECTED,
	CONNECTING,
	CONNECTED
};

enum SCALER_STATUS {
	SCALER_ACTIVE = 1,
	NO_SIGNAL,
	NO_SUPPORT
};

enum BATTERY_TEMP_STATUS {
	TEMP_NORMAL,
	TEMP_HOT,
	TEMP_FROZEN
};

struct scaler_pdata {
	int hpd_irq_gpio;		/*GPIO-112*/
	int scaler_irq_gpio;		/*GPIO-118*/
	int bl_up_gpio;			/*GPIO-99*/
	int bl_down_gpio;		/*GPIO-82*/
	int fm_update_gpio;	/*GPIO-109*/
	int sdram_en_gpio;		/*GPIO-22*/
	int vcc3v3_on_gpio;	/*GPIO-103*/
	int v11s_on_gpio;		/*GPIO-66*/
	int vdd_1v2_gpio;		/*GPIO-10*/
};

struct scaler_reg_status {
	enum SCALER_STATUS scaler_init_status;
	u8 request_PA_on;
	u8 request_scaler_off;
};

enum OSD_TYPE {
	BRIGHTNESS_ON = 1,
	BRIGHTNESS_OFF,
	BATTERY_CHARGING,
	BATTERY_NOCHARGE,
	MAX_OSD_TYPE
};

enum HDCP_KEY_STATUS {
	UNKONWN,
	FAIL,
	PASS
};

struct touch_info_t {
    int x;
    int y;
    bool long_press;
};

struct scaler_data {
	struct scaler_pdata *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct class *scaler_class;
	struct device *scaler_dev;
	struct notifier_block drm_notifier;
	dev_t scaler_devt;

	struct scaler_reg_status reg_status;
	enum CONNECTION_STATUS connection_status;
	struct mutex mutex;
	struct work_struct hpd_irq_work;
	struct work_struct scaler_irq_work;
	struct work_struct touch_irq_work;
	struct delayed_work update_status_work;
	struct delayed_work cable_detection_work;

    struct touch_info_t touch_info;

	int hpd_irq;
	int scaler_irq;

	u8 brightness;			// 0~255
	u8 brightness_step;		// 0~15
	u8 battery_capacity;		// 0~100
	u8 battery_temp;		// 0:NORMAL, 1:HOT, 2:FROZEN
	bool battery_charging;
	u8 volume_level;			// 0~15

	u8 fw_version;				// firmware version
	bool cable_connected;	// hdmi cable connected
	enum HDCP_KEY_STATUS hdcp_key_status;	// hdcp key status
	u8 hpd_gpio_status;
	bool panel_off;
};

extern enum CONNECTION_STATUS get_connection_status(void);
extern int notify_scaler_show_osd(enum OSD_TYPE type);
extern int notify_scaler_backlight_up_or_down(bool up, bool long_press);
extern int notify_scaler_touch_point(int x, int y, bool long_press);

#endif
