#ifndef __LINUX_FT5X06_TS_H__
#define __LINUX_FT5X06_TS_H__

#define FT5X06_NAME	"FT5X06_ts"
#define CFG_FTS_CTP_DRIVER_VERSION "1.0"


#define CFG_SUPPORT_AUTO_UPG 0
#define CFG_SUPPORT_UPDATE_PROJECT_SETTING  0
#define CFG_SUPPORT_TOUCH_KEY  1    //touch key, HOME, SEARCH, RETURN etc
#define CFG_NUMOFKEYS 4     


#define CFG_MAX_TOUCH_POINTS  5
#define CONFIG_FT5X06_MULTITOUCH 5 


#define KEY_PRESS       1
#define KEY_RELEASE     0


#define SCREEN_MAX_X    1024
#define SCREEN_MAX_Y    600
#define PRESS_MAX       255

#define FT5x06_TX_NUM	28
#define FT5x06_RX_NUM   16


#define CFG_POINT_READ_BUF  (3 + 6 * (CFG_MAX_TOUCH_POINTS))



enum FT5X06_ts_regs {
	FT5X06_REG_THGROUP					= 0x80,     /* touch threshold, related to sensitivity */
	FT5X06_REG_THPEAK					= 0x81,
	FT5X06_REG_THCAL					= 0x82,
	FT5X06_REG_THWATER					= 0x83,
	FT5X06_REG_THTEMP					= 0x84,
	FT5X06_REG_THDIFF					= 0x85,				
	FT5X06_REG_CTRL						= 0x86,
	FT5X06_REG_TIMEENTERMONITOR			= 0x87,
	FT5X06_REG_PERIODACTIVE				= 0x88,      /* report rate */
	FT5X06_REG_PERIODMONITOR			= 0x89,
	FT5X06_REG_HEIGHT_B					= 0x8a,
	FT5X06_REG_MAX_FRAME				= 0x8b,
	FT5X06_REG_DIST_MOVE				= 0x8c,
	FT5X06_REG_DIST_POINT				= 0x8d,
	FT5X06_REG_FEG_FRAME				= 0x8e,
	FT5X06_REG_SINGLE_CLICK_OFFSET		= 0x8f,
	FT5X06_REG_DOUBLE_CLICK_TIME_MIN	= 0x90,
	FT5X06_REG_SINGLE_CLICK_TIME		= 0x91,
	FT5X06_REG_LEFT_RIGHT_OFFSET		= 0x92,
	FT5X06_REG_UP_DOWN_OFFSET			= 0x93,
	FT5X06_REG_DISTANCE_LEFT_RIGHT		= 0x94,
	FT5X06_REG_DISTANCE_UP_DOWN		    = 0x95,
	FT5X06_REG_ZOOM_DIS_SQR				= 0x96,
	FT5X06_REG_RADIAN_VALUE				=0x97,
	FT5X06_REG_MAX_X_HIGH                       = 0x98,
	FT5X06_REG_MAX_X_LOW             			= 0x99,
	FT5X06_REG_MAX_Y_HIGH            			= 0x9a,
	FT5X06_REG_MAX_Y_LOW             			= 0x9b,
	FT5X06_REG_K_X_HIGH            			= 0x9c,
	FT5X06_REG_K_X_LOW             			= 0x9d,
	FT5X06_REG_K_Y_HIGH            			= 0x9e,
	FT5X06_REG_K_Y_LOW             			= 0x9f,
	FT5X06_REG_AUTO_CLB_MODE			= 0xa0,
	FT5X06_REG_LIB_VERSION_H 				= 0xa1,
	FT5X06_REG_LIB_VERSION_L 				= 0xa2,		
	FT5X06_REG_CIPHER						= 0xa3,
	FT5X06_REG_MODE					       = 0xa4,
	FT5X06_REG_PMODE						= 0xa5,	  /* Power Consume Mode		*/	
	FT5X06_REG_FIRMID						= 0xa6,   /* Firmware version */
	FT5X06_REG_STATE						= 0xa7,
	FT5X06_REG_FT5201ID					= 0xa8,
	FT5X06_REG_ERR						= 0xa9,
	FT5X06_REG_CLB						= 0xaa,
};

//Firmware ID register address
#define FT5x06_REG_FW_VER 0xa6


//FT5X06_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03


#ifndef ABS_MT_TOUCH_MAJOR
#define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
#define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
#define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
#define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
#define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
#define ABS_MT_POSITION_X	0x35	/* Center X ellipse position */
#define ABS_MT_POSITION_Y	0x36	/* Center Y ellipse position */
#define ABS_MT_TOOL_TYPE	0x37	/* Type of touching device */
#define ABS_MT_BLOB_ID		0x38	/* Group set of pkts as blob */
#endif /* ABS_MT_TOUCH_MAJOR */

#ifndef ABS_MT_TRACKING_ID
#define ABS_MT_TRACKING_ID 0x39 /* Unique ID of initiated contact */
#endif


struct ts_event {
    u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
    u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
    u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
    u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
    u16	pressure;
    u8  touch_point;
};


struct ft5x06_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
    struct timer_list ft5x06_timer;
    wait_queue_head_t waiter;
	int sample_ratio_per_HZ;
	struct mutex device_mode_mutex;   /* Ensures that only one function can specify the Device Mode at a time. */
	int tpd_flag;
};


#define SAMPLE_RATIO_PER_HZ 10

#define FT5x06_INT_PORT 17
#define FT5x06_INT_PIN 4

#define FT5x06_RST_PORT 5
#define FT5x06_RST_PIN 1

#endif
