/* drivers/input/touchscreen/ftxxxx_ts.c
*
* FocalTech ftxxxx TouchScreen driver.
*
* Copyright (c) 2014  Focaltech Ltd.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/i2c.h>
#include <linux/input.h>
/*#include <linux/earlysuspend.h>*/
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>/*ori #include <mach/irqs.h>*/
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include "ftxxxx_ts.h"
#include <linux/switch.h>
#include <linux/gpio.h>/*ori #include <mach/gpio.h>*/
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
/*#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
*/
/*#include "linux/input/proximity_class.h"*/
/*#include <linux/ft3x17.h>*/
#define SYSFS_DEBUG
/*#define FTS_SELF_TEST		not support now*/
#define FTS_APK_DEBUG		
#define FTS_PM
#define FTS_CTL_IIC 

#ifdef ASUS_FACTORY_BUILD

#else
#define FTS_GESTRUE
#endif
/*#define FTXXXX_ENABLE_IRQ*/

/*#define CONFIG_PM*/

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#define PROC_AsusTouchDisable_name	"asus_touch_proximity_status"

#ifdef FTS_GESTRUE
/*zax 20140922*/
#define KEY_GESTURE_U		KEY_POWER
#define KEY_GESTURE_UP		KEY_UP
#define KEY_GESTURE_DOWN		KEY_DOWN
#define KEY_GESTURE_LEFT		KEY_LEFT
#define KEY_GESTURE_RIGHT		KEY_RIGHT
#define KEY_GESTURE_M		KEY_M
#define KEY_GESTURE_L		KEY_L
/*asus use*/
#define KEY_GESTURE_V		KEY_V
#define KEY_GESTURE_Z		KEY_Z
#define KEY_GESTURE_C		KEY_C
#define KEY_GESTURE_E		KEY_E
#define KEY_GESTURE_S		KEY_S
#define KEY_GESTURE_W		KEY_W
/*asus use*/

#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP			0x22
#define GESTURE_DOWN		0x23
#define GESTURE_M			0x32
#define GESTURE_L			0x44
#define GESTURE_S			0x46
#define GESTURE_V			0x54
#define GESTURE_Z			0x65
#define GESTURE_C			0x34
#define GESTURE_E			0x33
#define GESTURE_O			0x30
#define GESTURE_W			0x31

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME 62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

int gestrue_id = 0;
#endif

#ifdef SYSFS_DEBUG
#include "ftxxxx_ex_fun.h"
#endif

/*ASUS_BSP jacob kung: add for debug mask +++ */
#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define focal_debug(level, ...) do { if (debug >= (level)) pr_info(__VA_ARGS__); } while (0)
/*ASUS_BSP jacob kung: add for debug mask --- */

volatile bool suspend_resume_process;
static bool disable_tp_flag;
int focal_init_success = 0;
bool FOCAL_IRQ_DISABLE = true;
int TPID = -1;
int Focal_hw_id = -1;
unsigned char IC_FW;
u8 B_VenderID;
u8 F_VenderID;
char B_projectcode[8];
u8 F_projectcode;
u8 g_vendor_id = 0xFF;
int HD_Modle =0;
u8 FTS_gesture_register_d2;
u8 FTS_gesture_register_d5;
u8 FTS_gesture_register_d6;
u8 FTS_gesture_register_d7;

u8 key_record;
/* +++ asus jacob add for print touch location +++ */
int report_touch_locatoin_count[10];
/* --- asus jacob add for print touch location --- */

/* +++ asus jacob add for reduce log +++ */
int SkipTouchCount = 0;
/* --- asus jacob add for reduce log --- */

/* +++ asus jacob add for reconfig double tap parameter +++ */
bool ReConfigDoubleTap = false;
u8 g_touch_slop = 0;
u8 g_touch_distance = 0;
u8 g_time_gap = 0;
u8 g_int_time = 0;
/* --- asus jacob add for reconfig double tap parameter --- */

/* +++ asus jacob add for check proximity status +++ */
bool EnableProximityCheck = true;
/* --- asus jacob add for check proximity status --- */

bool g_bl_suspend = false;

#ifdef FTS_PM
void ftxxxx_ts_suspend(void);
void ftxxxx_ts_resume(void);
#endif
struct ftxxxx_ts_data *ftxxxx_ts;
static bool touch_down_up_status;

#define TOUCH_MAX_X						1080
#define TOUCH_MAX_Y						1920

#define ANDROID_INPUT_PROTOCOL_B

/* jacob add for i2c retey and if i2c error countor > 10 reset IC */
#define IICReadWriteRetryTime	3
static int IICErrorCountor = 0;
/* jacob add for i2c retey  and if i2c error countor > 10 reset IC */

/*#define FTXXXX_RESET_PIN	88//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_RESET_PIN_NAME	"ft5x46-rst"
/*#define FTXXXX_INT_PIN	62//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_INT_PIN_NAME	"ft5x46-int"
/*add for find TPID*/
#define FTXXXX_LCD0_PIN_NAME  "LCD0-pin"
#define FTXXXX_LCD1_PIN_NAME  "LCD1-pin"
#define FTXXXX_TP0_PIN_NAME  "TP0-pin"
#define FTXXXX_TP1_PIN_NAME  "TP1-pin"
extern bool proximity_check_status(void);
extern int get_audiomode(void);

/*
*ftxxxx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;
	int retry = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 2);

			if (ret >= 0)
				break;

			msleep(1);
		}
	} else {
		if(readlen>0){
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 1);

			if (ret >= 0)
				break;

			msleep(1);
		}
	}
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read error.  error code = %d \n", __func__, ret);
		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read/write error over 10 times !! \n", __func__);
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: excute reset IC process !! \n", __func__);
			queue_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work);
			return ret;
		}

		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}
/*write data by i2c*/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int retry = 0;
	if(writelen>0){
		struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);

		if (ret >= 0)
			break;

		msleep(1);
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c write error.  error code = %d \n", __func__, ret);

		IICErrorCountor += 1;

			if (IICErrorCountor >= 10) {
				dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read/write error over 10 times !! \n", __func__);
				dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: excute reset IC process !! \n", __func__);
				//ASUSEvtlog("[Touch] touch i2c read/write error over 10 times, reset IC \n");	
				printk("[Touch] touch i2c read/write error over 10 times, reset IC \n");
				queue_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work);
				return ret;
			}
		}
		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}

/*ASUS_BSP Jacob : add for creating virtual_key_maps +++*/
#if 0
#define MAX_LEN		200

static ssize_t focalTP_virtual_keys_register(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	char *virtual_keys = 	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":160:3000:130:35" "\n" \

				__stringify(EV_KEY) ":" __stringify(KEY_HOME) ":360:3000:130:35" "\n" \

				__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":560:3000:130:35" "\n" ;

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",	virtual_keys);

}

static struct kobj_attribute focalTP_virtual_keys_attr = {

	.attr = {
		.name = "virtualkeys.focal-touchscreen",
		.mode = S_IRWXU | S_IRWXG | S_IROTH,
	},

	.show = &focalTP_virtual_keys_register,

};

static struct attribute *virtual_key_properties_attrs[] = {

	&focalTP_virtual_keys_attr.attr,

	NULL

};

static struct attribute_group virtual_key_properties_attr_group = {

	.attrs = virtual_key_properties_attrs,

};

struct kobject *focal_virtual_key_properties_kobj;
#endif
/*ASUS_BSP Jacob : add for creating virtual_key_maps ---*/

u8 get_focal_tp_fw(void)
{
	u8 fwver = 0;

	if (ftxxxx_read_reg(ftxxxx_ts->client, FTXXXX_REG_FW_VER, &fwver) < 0)
		return -1;
	else
		return fwver;
}
u8 get_focal_ft_id(void)
{
	u8 ftid = 0;

	if (ftxxxx_read_reg(ftxxxx_ts->client, FTXXXX_REG_FT_ID, &ftid) < 0)
		return -1;
	else
		return ftid;
}
u8 get_focal_touch_id(void)
{
	u8 touchicid = 0;

	if (ftxxxx_read_reg(ftxxxx_ts->client, FTXXXX_REG_TOUCH_IC, &touchicid) < 0)
		return -1;
	else
		return touchicid;
}
static ssize_t focal_show_tpfwver(struct switch_dev *sdev, char *buf)
{
	int num_read_chars = 0;
	IC_FW = get_focal_tp_fw();
	g_vendor_id=ftxxxx_read_tp_id();
	if (IC_FW == 255) {
		printk("[Focal][Touch] %s :  read FW fail \n ", __func__);
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	} else {
		printk("[Focal][Touch] %s :  TP_ID = 0x%x touch FW = 0x%x\n ", __func__, g_vendor_id, IC_FW);
		num_read_chars = snprintf(buf, PAGE_SIZE, "0x%x-0x%x\n", g_vendor_id, IC_FW);
	}
	return num_read_chars;
}

#ifdef FTS_GESTRUE/*zax 20140922*/
static void check_gesture(struct ftxxxx_ts_data *data, int gesture_id)
{
	bool Ps_status = false;

	printk("[Focal][Touch] %s :  gesture_id = 0x%x\n ", __func__, gesture_id);

	if ((EnableProximityCheck && !ftxxxx_ts->cover_mode_eable) && !(get_audiomode()==2))
	{	
		Ps_status = proximity_check_status();
		printk("[touch]check proximity status  \n");
	}
	if (!Ps_status) {
	switch (gesture_id) {
		/* ++++ touch gesture mode support part  ++++ */
		case GESTURE_DOUBLECLICK:
			input_report_key(data->input_dev, KEY_GESTURE_U, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_U, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_UP:
			input_report_key(data->input_dev, KEY_GESTURE_UP, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_UP, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_V:
			input_report_key(data->input_dev, KEY_GESTURE_V, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_V, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_Z:
			input_report_key(data->input_dev, KEY_GESTURE_Z, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_Z, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_E:
			input_report_key(data->input_dev, KEY_GESTURE_E, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_E, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_C:
			input_report_key(data->input_dev, KEY_GESTURE_C, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_C, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_S:
			input_report_key(data->input_dev, KEY_GESTURE_S, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_S, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_W:
			input_report_key(data->input_dev, KEY_GESTURE_W, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_W, 0);
			input_sync(data->input_dev);
			break;
		/* ---- touch gesture mode support part in ZE500CL ---- */

		/* ++++ touch gesture mode not support part in ZE500CL ++++ */
		/*
		case GESTURE_LEFT:
			input_report_key(data->input_dev, KEY_GESTURE_LEFT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_LEFT, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_RIGHT:
			input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_UP:
			input_report_key(data->input_dev, KEY_GESTURE_UP, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_UP, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_DOWN:
			input_report_key(data->input_dev, KEY_GESTURE_DOWN, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_DOWN, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_M:

			input_report_key(data->input_dev, KEY_GESTURE_M, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_M, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_L:
			input_report_key(data->input_dev, KEY_GESTURE_L, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_L, 0);
			input_sync(data->input_dev);
			break;

		case GESTURE_S:
			input_report_key(data->input_dev, KEY_GESTURE_S, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_GESTURE_S, 0);
			input_sync(data->input_dev);
			break;

		*/
		/* ---- touch gesture mode not support part in ZE500CL ++++ */
		default:

		break;

	}
	} else {
		printk("[Focal][Touch] %s :  Skip wake up devices !\n ", __func__);
	  }
		}


static int fts_read_Gestruedata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	int gestrue_id = 0;
	buf[0] = 0xd3;

	pointnum = 0;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	printk("[Focal][Touch] %s : tpd read FTS_GESTRUE_POINTS_HEADER.\n", __func__);

	if (ret < 0) {
		printk("[Focal][TOUCH_ERR] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	/*if (fts_updateinfo_curr.CHIP_ID==0x54)*/
	/*{*/
			 gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if ((pointnum * 4 + 8) < 255) {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
		} else {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 255);
			ret = ftxxxx_i2c_Read(data->client, buf, 0, buf+255, (pointnum * 4 + 8) - 255);
		}
		if ((ret < 0) | suspend_resume_process) {
			printk("[Focal][TOUCH_ERR] %s read touchdata failed.\n", __func__);
			return ret;
		}
	check_gesture(data, gestrue_id);
	for (i = 0; i < pointnum; i++) {
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[1 + (4 * i)]) & 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;
/*}*/
/*
	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		check_gesture(gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;

	if((pointnum * 4 + 8)<255) {
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	}
	else
	{
		 ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
		 ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf, pointnum);
	check_gesture(gestrue_id);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;*/
}
#endif

/*static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 8);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
	Read two times
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (8));
	ret = ftxxxx_i2c_Read(data->client, buf, 0, (buf+8), (8));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf,pointnum);

	printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}

	return -1;
}*/

/*Read touch point information when the interrupt  is asserted.*/
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	/*u8 force_threshold=0,reg_value=0;
	int gesture_id=0;*/
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "[Focal][TOUCH_ERR] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	focal_debug(DEBUG_VERBOSE, "[Focal][debug] read touch data ! \n");

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID) {
			focal_debug(DEBUG_VERBOSE, "[Focal][debug] pointid = %d ! \n", pointid);
			break;
		}
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		event->pressure[i] =
			(buf[FT_TOUCH_XY_POS + FT_TOUCH_STEP * i]);
		event->area[i] =
			(buf[FT_TOUCH_MISC + FT_TOUCH_STEP * i]) ;
		/*ftxxxx_read_reg(data->client, 0xff,&force_threshold);
		printk("[touch] force threshold = %x \n",force_threshold);
		if(event->pressure[i]>force_threshold){
			ftxxxx_read_reg(data->client, 0xfe,&reg_value);
			printk("[touch] gesture id from 0xfe = %x \n",reg_value);
			gesture_id = reg_value;
			printk("[touch] gesture id = %d \n",gesture_id);
			check_gesture(data, gesture_id);
			ftxxxx_write_reg(data->client, 0xfe, 0);
			}*/
			
		/*if(event->area[i]>0)  
			event->pressure[i] = event->area[i];*/
			/*event->pressure[i] =0xff;*/

		focal_debug(DEBUG_VERBOSE, "id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
	}

	/*event->pressure = FT_PRESS;*/
	/*event->pressure = 200;*/

	return 0;
}

/*
*report the point information
*/
static void ftxxxx_report_value(struct ftxxxx_ts_data *data)
{	
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	if (g_bl_suspend == true  )
		return;		
	/*protocol B*/
	for (i = 0; i < event->touch_point; i++) {
		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
			if ((event->au16_y[i] > 2980) & (event->au16_y[i] < 3020)) {
				if ((event->au16_x[i] > 140) & (event->au16_x[i] < 180)) {
					input_report_key(data->input_dev, KEY_BACK, 1);
					key_record |= 1;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d BACK KEY Press, key record = %d ! \n", event->au8_finger_id[i], key_record);
				} else if ((event->au16_x[i] > 340) & (event->au16_x[i] < 380)) {
					input_report_key(data->input_dev, KEY_HOME, 1);
					key_record |= 1 << 1;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d HOME KEY Press, key record = %d ! \n", event->au8_finger_id[i], key_record);
				} else if ((event->au16_x[i] > 540) & (event->au16_x[i] < 580)) {
					input_report_key(data->input_dev, KEY_MENU, 1);
					key_record |= 1 << 2;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d MENU KEY Press, key record = %d ! \n", event->au8_finger_id[i], key_record);
				}
			} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);             /* touch down*/
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]); /*ID of touched point*/
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			/*
			if(i>1){
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 16);//event->area[i]);
				}
			else
			{
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 16);//event->area[i]);
			}*/
			/* +++ asus jacob add for print touch location +++ */
			
			
			report_touch_locatoin_count[i] += 1;
			if ((report_touch_locatoin_count[i] % 200) == 0) {
				printk("[Focal][Touch] id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
				event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
				report_touch_locatoin_count[i] = 1;
			}
		}
			/* --- asus jacob add for print touch location --- */
			input_mt_sync(data->input_dev);
			/*printk("[Focal][Touch] report_abs_X = %d, report_abs_Y = %d  !\n", event->au16_x[i], event->au16_y[i]);*/

		} else {
			if ((event->au16_y[i] > 2980) & (event->au16_y[i] < 3020)) {
				if ((event->au16_x[i] > 140) & (event->au16_x[i] < 180)) {
					input_report_key(data->input_dev, KEY_BACK, 0);
					key_record &= 0xfe;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d BACK KEY Up ! \n", event->au8_finger_id[i]);
				} else if ((event->au16_x[i] > 340) & (event->au16_x[i] < 380)) {
					input_report_key(data->input_dev, KEY_HOME, 0);
					key_record &= 0xfd;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d HOME KEY Up ! \n", event->au8_finger_id[i]);
				} else if ((event->au16_x[i] > 540) & (event->au16_x[i] < 580)) {
					input_report_key(data->input_dev, KEY_MENU, 0);
					key_record &= 0xfb;
					focal_debug(DEBUG_VERBOSE, "[Focal][Touch][KEY] id=%d MENU KEY Up ! \n", event->au8_finger_id[i]);
				}
			}
			uppoint++;
			/* +++ asus jacob add for print touch location +++ */
			report_touch_locatoin_count[i] = 0;
			/* --- asus jacob add for print touch location --- */
			input_mt_sync(data->input_dev);
		}
	}

	if(event->touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		if (key_record) {
			if (key_record & 0x1) {
				input_report_key(data->input_dev, KEY_BACK, 0);
			}
			if (key_record & 0x2) {
				input_report_key(data->input_dev, KEY_HOME, 0);
			}
			if (key_record & 0x4) {
				input_report_key(data->input_dev, KEY_MENU, 0);
			}
		}
		key_record = 0;
		touch_down_up_status = 0;
		/* +++ asus jacob add for print touch location +++ */
		memset(report_touch_locatoin_count, 0, sizeof(report_touch_locatoin_count));
		/* --- asus jacob add for print touch location --- */
		printk("[Focal][Touch] touch up !\n");
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
		if (touch_down_up_status == 0) {
			touch_down_up_status = 1;
			printk("[Focal][Touch] touch down !\n");
			printk("[Focal][Touch] id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[0],
			event->au8_touch_event[0], event->au16_x[0], event->au16_y[0], event->pressure[0], event->area[0]);
		}
	}
	input_sync(data->input_dev);
}

/*The ftxxxx device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ftxxxx_ts_interrupt(int irq, void *dev_id)
{
/*	struct ftxxxx_ts_data *ftxxxx_ts = dev_id; ASUS jacob use globle ftxxxx_ts data*/
	int ret = 0;
#ifdef FTS_GESTRUE/*zax 20140922*/
	u8 state;
#endif
	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	ftxxxx_nosync_irq_disable(ftxxxx_ts->client);
#ifdef FTS_GESTRUE/*zax 20140922*/
				i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0xd0, 1, &state);
				/*printk("tpd fts_read_Gestruedata state=%d\n", state);*/
				if (state == 1) {
					fts_read_Gestruedata(ftxxxx_ts);
					/*continue;*/
				} else {
#endif
	ret = ftxxxx_read_Touchdata(ftxxxx_ts);

	if ((ret == 0) && (atomic_read(&ftxxxx_ts->irq_ref_cnt) == 1) && (suspend_resume_process == false) && (!disable_tp_flag)) {
		ftxxxx_report_value(ftxxxx_ts);
		if (SkipTouchCount)
			SkipTouchCount = 0;
	} else {
		if (((SkipTouchCount%200) == 0) || SkipTouchCount == 1)
			printk("[Focal][Interrupt] skip report touch !\n");
		SkipTouchCount++;
	}
#ifdef FTS_GESTRUE/*zax 20140922*/
					}
#endif

	ftxxxx_irq_enable(ftxxxx_ts->client);

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return IRQ_HANDLED;
}

void ftxxxx_reset_tp(int HighOrLow)
{
	pr_info("[Focal][Touch] %s : set tp reset pin to %d\n", __func__, HighOrLow);
	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, HighOrLow);
}

/* +++ Jacob add for proximity trigger disable touch +++ */
void ftxxxx_disable_touch(bool flag)
{
	if (flag) {
		disable_tp_flag = true;
		printk("[Focal][Touch] %s: proximity trigger disable touch !\n", __func__);
	} else {
		disable_tp_flag = false;
		printk("[Focal][Touch] %s: proximity trigger enable touch  !\n", __func__);
	}
}
EXPORT_SYMBOL(ftxxxx_disable_touch);

/* --- Jacob add for proximity trigger disable touch --- */
void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable)
{
	//if (FTXXXX_ENABLE_IRQ == enable)
	//if (FTXXXX_ENABLE_IRQ)
	//enable_irq(client->irq);
	//else
	//disable_irq_nosync(client->irq);
}

void ftxxxx_nosync_irq_disable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);
	
    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	atomic_inc(&ftxxxx_ts->irq_ref_cnt);	//asus jacob add for check irq dis/enable status
    if (!ts_data->irq_lock_status) {
		disable_irq_nosync(ts_data->client->irq);
		ts_data->irq_lock_status = 1;
	} else {
		printk("[Focal][Touch] %s : already disnable skip ! \n", __func__);
	}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void ftxxxx_irq_disable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);
	
    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	atomic_inc(&ftxxxx_ts->irq_ref_cnt);	//asus jacob add for check irq dis/enable status
    if (!ts_data->irq_lock_status) {
		disable_irq(ts_data->client->irq);
        ts_data->irq_lock_status = 1;
	} else {
		printk("[Focal][Touch] %s : already disnable skip ! \n", __func__);
	}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void ftxxxx_irq_enable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);

    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	atomic_dec(&ftxxxx_ts->irq_ref_cnt);	//asus jacob add for check irq dis/enable status
	if (!FOCAL_IRQ_DISABLE) {
		if (atomic_read(&ftxxxx_ts->irq_ref_cnt) <= 0) {
			if (ts_data->irq_lock_status) {
				enable_irq(ts_data->client->irq);
		        ts_data->irq_lock_status = 0;
			} else {
			printk("[Focal][Touch] %s : already enable skip ! \n", __func__);
			}
			atomic_set(&ftxxxx_ts->irq_ref_cnt, 0);			
		}
	}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

int ftxxxx_read_tp_id(void)
{

	int err = 0;
	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	ftxxxx_nosync_irq_disable(ftxxxx_ts->client);
	if (HD_Modle==2){
		err = fts_ctpm_fw_upgrade_ReadVendorID(ftxxxx_ts->client, &B_VenderID);
		printk("[touch]bootloader vender id= %x \n",B_VenderID);
		
	
		if(g_asus_lcdID==1){ //CTC (Libra 5.5)
			B_VenderID=(B_VenderID & 0xf0)|0x9;
			printk("[touch]read lcdid=1(Libra-CTC), vender id= %x \n",B_VenderID);
		}else if (g_asus_lcdID==2 || g_asus_lcdID==3){ //TM (Libra 5.5)
			B_VenderID=(B_VenderID & 0xf0)|0x1;
			printk("[touch]read lcdid=2 or 3 (TM), vender id= %x \n",B_VenderID);
		}else if (g_asus_lcdID==0){//BOE (Leo 5.2)
			B_VenderID=(B_VenderID & 0xf0)|0x4;
			printk("[touch]read lcdid= 0 (Leo-BOE), vender id= %x \n",B_VenderID);

			}
		if (err < 0)
			B_VenderID = 0xFF;
	}
	else{
		if(gpio_get_value(ftxxxx_ts->gpio_tp1id)==1){
			if (gpio_get_value(ftxxxx_ts->gpio_lcd1id)==1)
				B_VenderID=0x61;
			else if (gpio_get_value(ftxxxx_ts->gpio_lcd1id)==0)
				B_VenderID=0x63;
			}
		else if(gpio_get_value(ftxxxx_ts->gpio_tp1id)==0)
			{
			if (gpio_get_value(ftxxxx_ts->gpio_lcd1id)==1)
				B_VenderID=0x81;
			else if (gpio_get_value(ftxxxx_ts->gpio_lcd1id)==0)
				B_VenderID = 0x83;
			}
		else
			B_VenderID = 0xFF;
	}
	printk("[Focal][Touch] %s : TP Bootloadr info : vendor ID = %x !\n", __func__, B_VenderID);

	asus_check_touch_mode();

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	ftxxxx_irq_enable(ftxxxx_ts->client);

	return B_VenderID;
}

int focal_get_HW_ID(void)
{
	//Focal_hw_id =g_ASUS_hwID;
	printk("[Focal][Touch] %s : Focal get hw id %d !\n", __func__, Focal_hw_id);

	return Focal_hw_id;
}
void asus_check_touch_mode(void)
{
	uint8_t buf[2] = {0};	
	int err = 0;
	if (focal_init_success == 1) {

		if (ftxxxx_ts->usb_status == 1) {
			buf[0] = 0x8B;
			buf[1] = 0x01;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : switch usb mode fail ! \n", __func__);
		} else {
			buf[0] = 0x8B;
			buf[1] = 0x00;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : leave usb mode fail ! \n", __func__);
		}

		if (ftxxxx_ts->glove_mode_eable == 1) {
			buf[0] = 0xC0;
			buf[1] = 0x01;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : enable glove mode fail ! \n", __func__);
		} else {
			buf[0] = 0xC0;
			buf[1] = 0x00;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : disable glove mode fail ! \n", __func__);
		}

		if (ftxxxx_ts->cover_mode_eable == 1) {
			buf[0] = 0xC1;
			buf[1] = 0x01;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0) {
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			} else {
			buf[0] = 0xC3;
			buf[1] = 0x02;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : cover mode enable ! \n", __func__);
			}
		} else {
			buf[0] = 0xC1;
			buf[1] = 0x00;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0) {
				printk("[Focal][TOUCH_ERR] %s : cover mode disable fail ! \n", __func__);
			} else {
			buf[0] = 0xC3;
			buf[1] = 0x00	;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : cover mode disable fail ! \n", __func__);
			}
		}
	}
	return;
}

void focal_usb_detection(bool plugin)
{
	if (ftxxxx_ts == NULL) {
		printk("[Focal][TOUCH_ERR] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	if (ftxxxx_ts->init_success == 1) {
		if (plugin)
			ftxxxx_ts->usb_status = 1; /*AC plug in*/
		else
			ftxxxx_ts->usb_status = 0;	/*no AC */

		queue_work(ftxxxx_ts->usb_wq, &ftxxxx_ts->usb_detect_work);
	}
}

EXPORT_SYMBOL(focal_usb_detection);

static void focal_cable_status(struct work_struct *work)
{
	uint8_t buf[2] = {0};
	int status = ftxxxx_ts->usb_status;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	printk("[Focal][Touch] cable_status=%d, init_success=%d.\n", status, ftxxxx_ts->init_success);

	if (focal_init_success == 1) {
		if (status == 0) {	/*no AC */
			buf[0] = 0x8B;
			buf[1] = 0x00;
			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		} else if (status == 1) {	/*AC plug in*/
			buf[0] = 0x8B;
			buf[1] = 0x01;
			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		}
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;

}

static void focal_cover_mode_switch_work(struct work_struct *work)
{
	uint8_t buf[2] = {0};

	int err = 0;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (focal_init_success == 1) {
		if (ftxxxx_ts->cover_mode_eable) {

			buf[0] = 0xC1;
			buf[1] = 0x01;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0) {
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			} else {
			buf[0] = 0xC3;
			buf[1] = 0x02;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : cover mode enable ! \n", __func__);
			}

		} else {

			buf[0] = 0xC1;
			buf[1] = 0x00;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0) {
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			} else {
			buf[0] = 0xC3;
			buf[1] = 0x00;
			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : cover mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : cover mode disable ! \n", __func__);
			}
		}
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}


void focal_cover_switch(bool plugin)
{

	if (ftxxxx_ts == NULL) {
		printk("[Focal][TOUCH_ERR] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	if (ftxxxx_ts->init_success == 1) {
		if (plugin)
			ftxxxx_ts->cover_mode_eable = 1; /*glove mode enable*/
		else
			ftxxxx_ts->cover_mode_eable = 0;	/*glove mode disable*/

		queue_delayed_work(ftxxxx_ts->init_check_ic_wq, &ftxxxx_ts->cover_mode_switch_work, msecs_to_jiffies(10));
	}
	return;

}
static void focal_keyboard_mode_switch_work(struct work_struct *work)
{

	uint8_t buf[2] = {0};

	int err = 0;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (focal_init_success == 1) {
		if (ftxxxx_ts->keyboard_mode_eable) {

			buf[0] = 0xE8;

			buf[1] = 0x00;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : keyboard mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : keyboard mode enable ! \n", __func__);

		} else {

			buf[0] = 0xE8;

			buf[1] = 0x01;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : keyboard mode disable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : keyboard mode disable ! \n", __func__);

		}
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}

void focal_keyboard_switch(bool plugin)
{

	if (ftxxxx_ts == NULL) {
		printk("[Focal][TOUCH_ERR] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	if (ftxxxx_ts->init_success == 1) {
		if (plugin)
			ftxxxx_ts->keyboard_mode_eable = 1; /*keyboard display*/
		else
			ftxxxx_ts->keyboard_mode_eable = 0;	/*no keyboard */

		queue_delayed_work(ftxxxx_ts->init_check_ic_wq, &ftxxxx_ts->keyboard_mode_switch_work, msecs_to_jiffies(10));
	}
	return;

}
static void focal_glove_mode_switch_work(struct work_struct *work)
{

	uint8_t buf[2] = {0};

	int err = 0;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (focal_init_success == 1) {
		if (ftxxxx_ts->glove_mode_eable) {

			buf[0] = 0xC0;

			buf[1] = 0x01;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : glove mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : glove mode enable ! \n", __func__);

		} else {

			buf[0] = 0xC0;

			buf[1] = 0x00;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][TOUCH_ERR] %s : glove mode disable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : glove mode disable ! \n", __func__);

		}
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}

void focal_glove_switch(bool plugin)
{

	if (ftxxxx_ts == NULL) {
		printk("[Focal][TOUCH_ERR] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	if (ftxxxx_ts->init_success == 1) {
		if (plugin)
			ftxxxx_ts->glove_mode_eable = 1; /*glove mode enable*/
		else
			ftxxxx_ts->glove_mode_eable = 0;	/*glove mode disable*/

		queue_delayed_work(ftxxxx_ts->init_check_ic_wq, &ftxxxx_ts->glove_mode_switch_work, msecs_to_jiffies(10));
	}
	return;

}

static void focal_reset_ic_work(struct work_struct *work)
{

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	ftxxxx_nosync_irq_disable(ftxxxx_ts->client);

	ftxxxx_reset_tp(0);

	msleep(40);

	ftxxxx_reset_tp(1);

	msleep(200);

	IICErrorCountor = 0;

	asus_check_touch_mode();
	ftxxxx_irq_enable(ftxxxx_ts->client);

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;

}
#ifdef FTS_PM
static void focal_suspend_work(struct work_struct *work)
{
	uint8_t buf[2] = {0};
	bool need_irq_disable = false;	// add for judge irq need disable

	struct ftxxxx_ts_data *ts = ftxxxx_ts;

	suspend_resume_process = true;

	printk("[Focal][Touch] %s : Touch suspend +++ \n", __func__);

	ftxxxx_nosync_irq_disable(ts->client);

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] IC in suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		wake_unlock(&ftxxxx_ts->wake_lock);

		suspend_resume_process = false;

		ftxxxx_irq_enable(ts->client);

		return;
	}
	
#ifdef FTS_GESTRUE/*zax 20140922*/
	if ((ftxxxx_ts->dclick_mode_eable == 1) | (ftxxxx_ts->swipeup_mode_eable == 1) | (ftxxxx_ts->gesture_mode_eable == 1)) {
			printk("[Focal][Touch] %s : Touch gesture mode \n", __func__);

			ftxxxx_write_reg(ts->client, 0xd0, 0x01);

			if (ftxxxx_ts->dclick_mode_eable == 1) {
				printk("[Focal][Touch] %s : open dclick mode \n", __func__);
				if(ftxxxx_ts->swipeup_mode_eable == 1)
					ftxxxx_write_reg(ts->client, 0xd1, 0x14);
				else
					ftxxxx_write_reg(ts->client, 0xd1, 0x10);
				if (ReConfigDoubleTap) {
					if (g_touch_slop) {
						ftxxxx_write_reg(ts->client, 0xe2, g_touch_slop);
						focal_debug(DEBUG_VERBOSE, "[Focal][Touch] %s : Set E2 to 0x%x \n", __func__, g_touch_slop);
					} else {
						ftxxxx_write_reg(ts->client, 0xe2, 0x0f);
					}
					if (g_touch_distance) {
						ftxxxx_write_reg(ts->client, 0xe3, g_touch_distance);
						focal_debug(DEBUG_VERBOSE, "[Focal][Touch] %s : Set E3 to 0x%x \n", __func__, g_touch_distance);
					} else {
						ftxxxx_write_reg(ts->client, 0xe3, 0x11);
					}
					if (g_time_gap) {
						ftxxxx_write_reg(ts->client, 0xe4, g_time_gap);
						focal_debug(DEBUG_VERBOSE, "[Focal][Touch] %s : Set E4 to 0x%x \n", __func__, g_time_gap);
					}
					if (g_int_time) {
						ftxxxx_write_reg(ts->client, 0xe5, g_int_time);
						focal_debug(DEBUG_VERBOSE, "[Focal][Touch] %s : Set E5 to 0x%x \n", __func__, g_int_time);
					}
				} else {
					ftxxxx_write_reg(ts->client, 0xe2, 0x0f);
					ftxxxx_write_reg(ts->client, 0xe3, 0x11);
				}
			}else{
				if(ftxxxx_ts->swipeup_mode_eable == 1)
					ftxxxx_write_reg(ts->client, 0xd1, 0x04);
			}

			if (ftxxxx_ts->gesture_mode_eable == 1) {
				if (ftxxxx_ts->dclick_mode_eable == 1){
					if (ftxxxx_ts->swipeup_mode_eable == 1)  
						ftxxxx_write_reg(ts->client, 0xd1, 0x34);
					else
						ftxxxx_write_reg(ts->client, 0xd1, 0x30);
				}else if (ftxxxx_ts->swipeup_mode_eable == 1)
						ftxxxx_write_reg(ts->client, 0xd1, 0x24);
				else
					ftxxxx_write_reg(ts->client, 0xd1, 0x20);

				printk("[Focal][Touch] %s : open gesture mode d2 = %x d5 = %x d6 = %x d7 = %x \n",
					__func__, FTS_gesture_register_d2, FTS_gesture_register_d5, FTS_gesture_register_d6, FTS_gesture_register_d7);

				ftxxxx_write_reg(ts->client, 0xd2, FTS_gesture_register_d2);

				ftxxxx_write_reg(ts->client, 0xd5, FTS_gesture_register_d5);

				ftxxxx_write_reg(ts->client, 0xd6, FTS_gesture_register_d6);

				ftxxxx_write_reg(ts->client, 0xd7, FTS_gesture_register_d7);
			}

			need_irq_disable = false;
			/*
			ftxxxx_write_reg(ts->client, 0xd1, 0xff);
			ftxxxx_write_reg(ts->client, 0xd2, 0xff);
			ftxxxx_write_reg(ts->client, 0xd5, 0xff);
			ftxxxx_write_reg(ts->client, 0xd6, 0xff);
			ftxxxx_write_reg(ts->client, 0xd7, 0xff);
			ftxxxx_write_reg(ts->client, 0xd8, 0xff);
			*/
	} else {
		printk("[Focal][Touch] %s : Touch suspend \n", __func__);
		need_irq_disable = true;
		buf[0] = 0xA5;
		buf[1] = 0x03;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
	}
#else
	printk("[Focal][Touch] %s : Touch suspend \n", __func__);
	need_irq_disable = true;
	buf[0] = 0xA5;
	buf[1] = 0x03;
	ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
#endif
	/* +++ asus jacob add for print touch location +++ */
	memset(report_touch_locatoin_count, 0, sizeof(report_touch_locatoin_count));
	/* --- asus jacob add for print touch location --- */

	input_report_key(ftxxxx_ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ftxxxx_ts->input_dev);
	if (key_record) {
		if (key_record & 0x1) {
			input_report_key(ftxxxx_ts->input_dev, KEY_BACK, 0);
		}
		if (key_record & 0x2) {
			input_report_key(ftxxxx_ts->input_dev, KEY_HOME, 0);
		}
		if (key_record & 0x4) {
			input_report_key(ftxxxx_ts->input_dev, KEY_MENU, 0);
		}
	}
	key_record = 0;
	touch_down_up_status = 0;
	input_sync(ftxxxx_ts->input_dev);
	ftxxxx_ts->suspend_flag = 1;

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	if (!need_irq_disable) {
		ftxxxx_irq_enable(ts->client);
		enable_irq_wake(ts->client->irq);
		ftxxxx_ts->irq_wakeup_eable = 1;
	}
	suspend_resume_process = false;

	printk("[Focal][Touch] %s : irq_wake_up = %d Touch suspend --- \n", __func__, ftxxxx_ts->irq_wakeup_eable);

	return;
}
static void focal_resume_work(struct work_struct *work)
{
	uint8_t buf[2] = {0};
	struct ftxxxx_ts_data *ts = ftxxxx_ts;

	suspend_resume_process = true;

	disable_tp_flag = false;

	printk("[Focal][Touch] %s : Touch resume +++ \n", __func__);

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (!ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] IC did not enter suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		wake_unlock(&ftxxxx_ts->wake_lock);

		suspend_resume_process = false;

		return;

	}

#ifdef FTS_GESTRUE	/*zax 20140922*/

	if ((ftxxxx_ts->dclick_mode_eable == 1)|(ftxxxx_ts->swipeup_mode_eable == 1) | (ftxxxx_ts->gesture_mode_eable == 1)) {
		if (ftxxxx_ts->reset_pin_status == 1) {

			printk("[Focal][Touch] %s : Touch resume from gesture mode \n", __func__);

			gpio_set_value(ts->pdata->rst_gpio, 0);
			msleep(40);
//printk("[Focal][Touch] : reset pin= %s \n",&ts->pdata->rst_gpio);
		gpio_set_value(ts->pdata->rst_gpio, 1);

			msleep(200);
//printk("[Focal][Touch] : reset pin= %s \n",ts->pdata->rst_gpio );
			asus_check_touch_mode();

			ftxxxx_write_reg(ts->client, 0xD0, 0x00);

	} else {

		printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);

	}

	} else {

		if (ftxxxx_ts->reset_pin_status == 1) {

			printk("[Focal][Touch] %s : Touch resume from sleep mode \n", __func__);

			gpio_set_value(ts->pdata->rst_gpio, 0);

			msleep(40);

		gpio_set_value(ts->pdata->rst_gpio, 1);

			msleep(200);

			asus_check_touch_mode();

			buf[0] = 0xA5;

			buf[1] = 0x00;

			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

		} else {

			printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);


		}
	}
#else
	if (ftxxxx_ts->reset_pin_status == 1) {

		gpio_set_value(ts->pdata->rst_gpio, 0);
		printk("[Focal][Touch] :resume touch reset set to 0 !  \n");
		msleep(40);

		gpio_set_value(ts->pdata->rst_gpio, 1);
		printk("[Focal][Touch] : resume touch reset set to 1 !  \n");
		msleep(200);

		asus_check_touch_mode();

		buf[0] = 0xA5;

		buf[1] = 0x00;

		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
	} else {

		printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);

	}
#endif

	SkipTouchCount = 0;

	ftxxxx_ts->suspend_flag = 0;

	if (ftxxxx_ts->irq_wakeup_eable) {
		disable_irq_wake(ts->client->irq);
		ftxxxx_ts->irq_wakeup_eable = 0;
	}
	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	suspend_resume_process = false;

	ftxxxx_irq_enable(ts->client);

	printk("[Focal][Touch] %s : irq_wake_up = %d Touch resume --- \n", __func__, ftxxxx_ts->irq_wakeup_eable);
	
	return;
}
#endif

static void focal_init_check_ic_work(struct work_struct *work)
{

	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	int tmp_err = 0;

	printk("[Progress][Focal-Touch] WQ Start !\n");

	

	/*get some register information */
	uc_reg_addr = FTXXXX_REG_FW_VER;
	tmp_err = ftxxxx_i2c_Read(ftxxxx_ts->client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (tmp_err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[[Focal][Touch] Firmware version = 0x%x\n", uc_reg_value);
		IC_FW = uc_reg_value;
		}
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	tmp_err = ftxxxx_i2c_Read(ftxxxx_ts->client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (tmp_err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[[Focal][Touch] Vendor ID = 0x%x\n", uc_reg_value);
		F_VenderID = uc_reg_value;
		}
	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	tmp_err = ftxxxx_i2c_Read(ftxxxx_ts->client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (tmp_err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] report rate is %dHz.\n", uc_reg_value * 10);
		}

	uc_reg_addr = FTXXXX_REG_THGROUP;
	tmp_err = ftxxxx_i2c_Read(ftxxxx_ts->client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (tmp_err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] touch threshold is %d.\n", uc_reg_value * 4);
		}
/*
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	tmp_err = ftxxxx_i2c_Read(ftxxxx_ts->client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (tmp_err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] VENDOR ID = 0x%x\n", uc_reg_value);
		}*/
	/*if (ftxxxx_ts->init_success == 1) {
	tmp_err = focal_fw_auto_update(ftxxxx_ts->client);
	} else {
		printk("[Focal][TOUCH_ERR] init error, skip update FW !\n");
	}*/
	printk("[Focal][Touch] X-RES = %d, Y-RES = %d, RST gpio = %d, gpio irq = %d, client irq = %d , lcd0id_gpio =%d , lcd1id_gpio =%d \n",
		ftxxxx_ts->pdata->abs_x_max, ftxxxx_ts->pdata->abs_y_max, ftxxxx_ts->pdata->rst_gpio, ftxxxx_ts->irq, ftxxxx_ts->client->irq,ftxxxx_ts->gpio_lcd0id,ftxxxx_ts->gpio_lcd1id);
	if(HD_Modle==0)
		printk("[Focal][Touch] tp0id_gpio=%d ,tp1id_gpio=%d\n",ftxxxx_ts->gpio_tp0id,ftxxxx_ts->gpio_tp1id);

printk("[Focal][Touch] TPID= %x \n",ftxxxx_read_tp_id());
	if (ftxxxx_ts->init_success == 1) {
		focal_init_success = 1;
		//if (g_ASUS_hwID >= ZE500KL_SR1 )
			FOCAL_IRQ_DISABLE = false;
	}

	asus_check_touch_mode();

	ftxxxx_irq_enable(ftxxxx_ts->client);

	/*if (tmp_err) {
		
		//ASUSEvtlog("[Touch][ERROR] FW update error, reset IC \n");
		printk("[Touch][ERROR] FW update error, reset IC \n");
		queue_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work);

	}*/
	printk("[Progress][Focal-Touch] WQ ends !\n");

	return;

}


static int fts_power_on(struct ftxxxx_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->pdata->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->pdata->vcc);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->pdata->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->pdata->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->pdata->vcc);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->pdata->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int fts_power_init(struct device *dev,
			struct focal_i2c_platform_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
	/* +++ pars regulator+++ */
	pdata->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR( pdata->vdd)) {
		rc = PTR_ERR(pdata->vdd);
		printk("Regulator get touch vdd failed rc=%d\n", rc);
		return rc;
		}

	if (regulator_count_voltages(pdata->vdd) > 0) {
		rc = regulator_set_voltage(pdata->vdd, FT_VDD_MIN_UV,
					   FT_VDD_MAX_UV);
		if (rc) {
			printk("Regulator set_vdd failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	
	pdata->vcc = devm_regulator_get(dev, "vcc_i2c");
	if (IS_ERR( pdata->vcc)) {
		rc = PTR_ERR(pdata->vcc);
		printk("Regulator get vcc_i2c failed rc=%d\n", rc);
		goto reg_vdd_set_vtg;
		}

	if (regulator_count_voltages(pdata->vcc) > 0) {
		rc = regulator_set_voltage(pdata->vcc, FT_I2C_VCC_MIN_UV,
					   FT_I2C_VCC_MAX_UV);
		if (rc) {
			printk("Regulator set_vcc failed vdd rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	/* +++ pars regulator+++ */
	return 0;
reg_vcc_i2c_put:
	regulator_put(pdata->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, FT_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(pdata->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, FT_VDD_MAX_UV);

	regulator_put(pdata->vdd);

	if (regulator_count_voltages(pdata->vcc) > 0)
		regulator_set_voltage(pdata->vcc, 0, FT_I2C_VCC_MAX_UV);

	regulator_put(pdata->vcc);
	return 0;
}
/*static int fts_init_gpio_TP0ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;
	
	ret = gpio_request(ftxxxx_ts->pdata->tp0id_gpio, FTXXXX_TP0_PIN_NAME);
	
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for tp0id failed, ret = %d\n",
			__func__, FTXXXX_TP0_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_input(ftxxxx_ts->pdata->tp0id_gpio);	//asus change reset set output high
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_TP0_PIN_NAME, ret);
		return ret;
		}
	
	return ret;
}
static int fts_init_gpio_TP1ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(ftxxxx_ts->pdata->tp1id_gpio, FTXXXX_TP1_PIN_NAME);
	
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for tp1id failed, ret = %d\n",
			__func__, FTXXXX_TP1_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_input(ftxxxx_ts->pdata->tp1id_gpio);	//asus change reset set output high
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_TP1_PIN_NAME, ret);
		return ret;
		}
	
	return ret;
}
static int fts_init_gpio_LCD0ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(ftxxxx_ts->pdata->lcd0id_gpio, FTXXXX_LCD0_PIN_NAME);
	
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for lcd0id failed, ret = %d\n",
			__func__, FTXXXX_LCD0_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_input(ftxxxx_ts->pdata->lcd0id_gpio);	//asus change reset set output high
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_LCD0_PIN_NAME, ret);
		return ret;
		}
	
	return ret;
}

static int fts_init_gpio_LCD1ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(ftxxxx_ts->pdata->lcd1id_gpio, FTXXXX_LCD1_PIN_NAME);
	
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for lcd1id failed, ret = %d\n",
			__func__, FTXXXX_LCD1_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_input(ftxxxx_ts->pdata->lcd1id_gpio);	//asus change reset set output high
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_LCD1_PIN_NAME, ret);
		return ret;
		}
	
	return ret;
}*/
static int fts_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(ftxxxx_ts->pdata->rst_gpio, FTXXXX_RESET_PIN_NAME);
	
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_output(ftxxxx_ts->pdata->rst_gpio, 1);	/*asus change reset set output high*/
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
		}
	return ret;
}

static void fts_un_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
}
/*
static void fts_un_init_gpio_TP0ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->tp0id_gpio);
}
static void fts_un_init_gpio_TP1ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->tp1id_gpio);
}
static void fts_un_init_gpio_LCD0ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->lcd0id_gpio);
}
static void fts_un_init_gpio_LCD1ID(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->lcd1id_gpio);
}*/
/* +++asus jacob add for pars dt info+++ */

static int ft5x46_get_dt_coords(struct device *dev, char *name,
				struct focal_i2c_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->abs_x_min = coords[0];
		pdata->abs_y_min = coords[1];
		pdata->abs_x_max = coords[2];
		pdata->abs_y_max = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->abs_x_min = coords[0];
		pdata->abs_y_min = coords[1];
		pdata->abs_x_max = coords[2];
		pdata->abs_y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x46_parse_dt(struct device *dev,
			struct focal_i2c_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;

/*
	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", *pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
*/
	rc = ft5x46_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;
	
	/* +++reset, irq gpio info+++ */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->rst_gpio_flag);
	if (pdata->rst_gpio < 0)
		return pdata->rst_gpio;

	pdata->intr_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->intr_gpio_flag);
	if (pdata->intr_gpio < 0)
		return pdata->intr_gpio;
	/* ---reset, irq gpio info--- */
	/*++tpid gpio info+++*/
	pdata->lcd0id_gpio=of_get_named_gpio_flags(np, "focaltech,lcdid0-gpio",
				0, &pdata->lcd0id_gpio_flag);
	if (pdata->lcd0id_gpio < 0)
		return pdata->lcd0id_gpio;
	pdata->lcd1id_gpio=of_get_named_gpio_flags(np, "focaltech,lcdid1-gpio",
				0, &pdata->lcd1id_gpio_flag);
	if (pdata->lcd1id_gpio < 0)
		return pdata->lcd1id_gpio;
	pdata->tp0id_gpio=of_get_named_gpio_flags(np, "focaltech,tpid0-gpio",
				0, &pdata->tp0id_gpio_flag);
	pdata->tp1id_gpio=of_get_named_gpio_flags(np, "focaltech,tpid1-gpio",
				0, &pdata->tp1id_gpio_flag);
	if (!gpio_is_valid(pdata->tp1id_gpio)){
		//printk("[Focal][Touch] didn't read tpid! \n ");
		HD_Modle = 2;
		printk("[Focal][Touch]HD_Modle =2 ! \n ");
		return 0;
		}
	return 0;
}
/* ---asus jacob add for pars dt info--- */

/* +++ asus jacob add for amax disable/enable touch +++ */
static ssize_t amax_disable_touch_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
/*
	char msg[64];

	memset(msg, 0, sizeof(msg));

	if(len > 64)
		len = 64;
	if(copy_from_user(msg, buff, len))
		return -EFAULT;

	if(strncmp(msg, "0", 1) == 0) {	// enable touch
		disable_tp_flag = 0;
		printk("[Focal][Touch] %s : enable touch !\n", __func__);
	} else if(strncmp(msg, "1", 1) == 0){
		disable_tp_flag = 1;
		printk("[Focal][Touch] %s : disable touch !\n", __func__);
	} else {
		printk("[Focal][Touch] %s : Input parameter invaild !\n", __func__);
	}
*/
	return len;
}

static const struct file_operations amax_disable_touch_fops = {
	.write = amax_disable_touch_write,
};

/* --- asus jacob add for amax disable/enable touch --- */

static int ftxxxx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focal_i2c_platform_data *pdata = (struct focal_i2c_platform_data *)client->dev.platform_data;
	struct input_dev *input_dev;
	int err = 0;
/*
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
*/
	printk("[Focal][Touch] FTxxxx prob process Start !\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	}

	if (client->dev.of_node) {
		ftxxxx_ts = kzalloc(sizeof(struct ftxxxx_ts_data), GFP_KERNEL);
		if (!ftxxxx_ts) {
			err = -ENOMEM;
			printk("[Focal][TOUCH_ERR] %s: alloc ftxxxx_ts_data failed !! \n", __func__);
			goto exit_alloc_data_failed;
		}
		pdata = kzalloc(sizeof(struct focal_i2c_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			printk("[Focal][TOUCH_ERR] %s: alloc focal_i2c_platform_data failed !! \n", __func__);
			goto exit_alloc_data_failed;
		}
	}

	err = ft5x46_parse_dt(&client->dev, pdata);
	if (err) {
		dev_err(&client->dev, "DT parsing failed\n");
		return err;
	}

	i2c_set_clientdata(client, ftxxxx_ts);

	ftxxxx_ts->client = client;
	ftxxxx_ts->init_success = 0;
	ftxxxx_ts->suspend_flag = 0;
	ftxxxx_ts->usb_status = 0;
	ftxxxx_ts->glove_mode_eable = 0;
	ftxxxx_ts->cover_mode_eable = 0;
	ftxxxx_ts->keyboard_mode_eable=0;
	ftxxxx_ts->dclick_mode_eable = 0;
	ftxxxx_ts->swipeup_mode_eable = 0;
	ftxxxx_ts->gesture_mode_eable = 0;
	ftxxxx_ts->irq_wakeup_eable = 0;
	ftxxxx_ts->gesture_mode_type = 0;
	ftxxxx_ts->pdata = pdata;
	ftxxxx_ts->x_max = pdata->abs_x_max;
	ftxxxx_ts->y_max = pdata->abs_y_max;
	if (0 >= ftxxxx_ts->x_max)
		ftxxxx_ts->x_max = TOUCH_MAX_X;
	if (0 >= ftxxxx_ts->y_max)
		ftxxxx_ts->y_max = TOUCH_MAX_Y;
	ftxxxx_ts->irq = ftxxxx_ts->pdata->intr_gpio;

	if (fts_init_gpio_hw (ftxxxx_ts) < 0)
		goto exit_init_gpio;

	/*if(HD_Modle==0){
		printk("[Focal][Touch] HD_Modle =0 \n");
		if (fts_init_gpio_TP0ID (ftxxxx_ts) < 0)
			goto exit_init_gpio_TP0ID;
		if (fts_init_gpio_TP1ID (ftxxxx_ts) < 0)
			goto exit_init_gpio_TP1ID;
	}
	if (fts_init_gpio_LCD0ID(ftxxxx_ts) < 0)

	if (fts_init_gpio_LCD1ID(ftxxxx_ts) < 0)

	printk("[Focal][Touch] Nancy 1 \n");
	*/
	ftxxxx_ts->reset_pin_status = 1;
	atomic_set(&ftxxxx_ts->irq_ref_cnt, 0);
	/*gpio_set_value(ftxxxx_ts->pdata->lcd0id_gpio, 0);
	gpio_set_value(ftxxxx_ts->pdata->lcd1id_gpio, 0);
	if(HD_Modle==0){
	gpio_set_value(ftxxxx_ts->pdata->tp1id_gpio, 0);
	gpio_set_value(ftxxxx_ts->pdata->tp0id_gpio, 0);
	}*/
	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 0);

	msleep(40);

	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 1);
	//msleep(200);
	/*if(HD_Modle==0){
	gpio_set_value(ftxxxx_ts->pdata->tp0id_gpio, 1);
	gpio_set_value(ftxxxx_ts->pdata->tp1id_gpio, 1);
	}
	gpio_set_value(ftxxxx_ts->pdata->lcd0id_gpio, 1);

	gpio_set_value(ftxxxx_ts->pdata->lcd1id_gpio, 1);
*/

	if (fts_power_init(&client->dev, ftxxxx_ts->pdata, true) < 0)
		goto exit_init_gpio;

	if (fts_power_on(ftxxxx_ts, true) < 0)
		goto exit_power_init;

	if (gpio_request(ftxxxx_ts->irq, FTXXXX_INT_PIN_NAME)) {
		printk("[Focal][TOUCH_ERR] %s: gpio %d request for interrupt fail.\n", __func__, ftxxxx_ts->irq);
		goto exit_irq_request_failed;
	}
	gpio_direction_input(ftxxxx_ts->irq);


	ftxxxx_ts->client->irq = gpio_to_irq(ftxxxx_ts->irq);
	err = request_threaded_irq(ftxxxx_ts->client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		ftxxxx_ts);

	if (ftxxxx_ts->client->irq< 0) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: request irq fail. \n", __func__);
		goto exit_irq_request_failed;
	}


	spin_lock_init(&ftxxxx_ts->irq_lock);

	ftxxxx_irq_disable(ftxxxx_ts->client);	/*need mutex protect, should add latter*/

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}
	/*add for find TPID*/
	/*ftxxxx_ts->gpio_lcd0id=pdata->lcd0id_gpio;
	ftxxxx_ts->gpio_lcd1id=pdata->lcd1id_gpio;
	ftxxxx_ts->gpio_tp0id=pdata->tp0id_gpio;
	ftxxxx_ts->gpio_tp1id=pdata->tp1id_gpio;
*/
	ftxxxx_ts->input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ftxxxx_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ftxxxx_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);

	input_dev->name = Focal_input_dev_name;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: failed to register input device: %s\n", __func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */

#ifdef SYSFS_DEBUG
/*
	printk("[Focal][Touch] ftxxxx_create_sysfs Start !\n");
	ftxxxx_create_sysfs(client);
	printk("[Focal][Touch] ftxxxx_create_sysfs End !\n");
*/
	mutex_init(&ftxxxx_ts->g_device_mutex);

	wake_lock_init(&ftxxxx_ts->wake_lock, WAKE_LOCK_SUSPEND, "focal_touch_wake_lock");

	ftxxxx_ts->usb_wq = create_singlethread_workqueue("focal_usb_wq");
	if (!ftxxxx_ts->usb_wq) {
		printk("[Focal][TOUCH_ERR] %s: create usb workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->usb_detect_work, focal_cable_status);
#ifdef FTS_PM
	ftxxxx_ts->suspend_resume_wq = create_singlethread_workqueue("focal_suspend_resume_wq");
	if (!ftxxxx_ts->suspend_resume_wq) {
		printk("[Focal][TOUCH_ERR] %s: create resume workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->resume_work, focal_resume_work);

	INIT_WORK(&ftxxxx_ts->suspend_work, focal_suspend_work);
#endif
	ftxxxx_ts->reset_wq = create_singlethread_workqueue("focal_reset_ic_wq");
	if (!ftxxxx_ts->reset_wq) {
		printk("[Focal][TOUCH_ERR] %s: create reset ic workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->reset_ic_work, focal_reset_ic_work);
	ftxxxx_ts->init_check_ic_wq = create_singlethread_workqueue("focal_init_check_ic_wq");
	if (!ftxxxx_ts->init_check_ic_wq) {
		printk("[Focal][TOUCH_ERR] %s: create init_check_ic workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	
	INIT_DELAYED_WORK(&ftxxxx_ts->init_check_ic_work, focal_init_check_ic_work);

	INIT_DELAYED_WORK(&ftxxxx_ts->glove_mode_switch_work, focal_glove_mode_switch_work);
	
	INIT_DELAYED_WORK(&ftxxxx_ts->keyboard_mode_switch_work, focal_keyboard_mode_switch_work);
	
	INIT_DELAYED_WORK(&ftxxxx_ts->cover_mode_switch_work, focal_cover_mode_switch_work);

	ftxxxx_ts->touch_sdev.name = "touch";
	ftxxxx_ts->touch_sdev.print_name = focal_show_tpfwver;

	if (switch_dev_register(&ftxxxx_ts->touch_sdev) < 0)
	{
		printk("[Focal][TOUCH_ERR] %s: failed to register switch_dev \n", __func__);
		goto exit_err_sdev_register_fail;
	} 

	/*ASUS_BSP Jacob: add for creating virtual_key_maps +++*/
#if 0
	focal_virtual_key_properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (focal_virtual_key_properties_kobj)

		err = sysfs_create_group(focal_virtual_key_properties_kobj, &virtual_key_properties_attr_group);

	if (!focal_virtual_key_properties_kobj || err)

		printk("[Focal][TOUCH_ERR] %s : failed to create novaTP virtual key map! \n", __func__);
#endif
	/*ASUS_BSP Jacob: add for creating virtual_key_maps ---*/
#endif
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s : create fts control iic driver failed\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_create_apk_debug_channel(client);
#endif
	printk("[Focal][Touch] ftxxxx_create_sysfs Start !\n");
	ftxxxx_create_sysfs(ftxxxx_ts->client);
	printk("[Focal][Touch] ftxxxx_create_sysfs End !\n");
/* +++ asus jacob add for amax disable/enable touch +++ */
	proc_create(PROC_AsusTouchDisable_name, 0664, NULL, &amax_disable_touch_fops);
/* --- asus jacob add for amax disable/enable touch --- */

	/*get some register information move to delay workque*/
/*
	uc_reg_addr = FTXXXX_REG_FW_VER;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[[Focal][Touch] Firmware version = 0x%x\n", uc_reg_value);
		IC_FW = uc_reg_value;
		}

	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] report rate is %dHz.\n", uc_reg_value * 10);
		}

	uc_reg_addr = FTXXXX_REG_THGROUP;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] touch threshold is %d.\n", uc_reg_value * 4);
		}
		
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] VENDOR ID = 0x%x\n", uc_reg_value);
		}

	printk("[Focal][Touch] TP ID = %d \n", ftxxxx_read_tp_id());
*/
#ifdef FTS_GESTRUE	/*zax 20140922*/
	/*init_para(720,1280,100,0,0);*/

	/*auc_i2c_write_buf[0] = 0xd0;*/
	/*auc_i2c_write_buf[1] = 0x01;*/
	/*ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gestrue function*/

	/*auc_i2c_write_buf[0] = 0xd1;*/
	/*auc_i2c_write_buf[1] = 0xff;*/
	/*ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);*/
	/*
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x00;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw close gestrue function 
	*/

	/* ++++ touch gesture mode not support part in ZE552kL ++++ */
	/*
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);

	__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	__set_bit(KEY_GESTURE_U, input_dev->keybit);
	__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	*/
	/* ---- touch gesture mode not support part in ZE552kL ---- */

	/* ++++ touch gesture mode support part in ZE552kL ++++ */
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);

	__set_bit(KEY_POWER, input_dev->keybit);
	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	/* ---- touch gesture mode support part in ZE552kL ---- */
#endif

	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	input_set_capability(input_dev, EV_KEY, KEY_HOME);
	input_set_capability(input_dev, EV_KEY, KEY_MENU);

	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_HOME, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
	queue_delayed_work(ftxxxx_ts->init_check_ic_wq, &ftxxxx_ts->init_check_ic_work, msecs_to_jiffies(1000));

	printk("[Focal][Touch][INFO] client name = %s irq = %d\n", client->name, client->irq);

	/* +++move to delay workque+++ */
/*
	printk("[Focal][Touch] X-RES = %d, Y-RES = %d, RST gpio = %d, gpio irq = %d, client irq = %d\n",
		ftxxxx_ts->pdata->abs_x_max, ftxxxx_ts->pdata->abs_y_max, ftxxxx_ts->pdata->rst_gpio, ftxxxx_ts->irq, ftxxxx_ts->client->irq);

	if (ftxxxx_ts->init_success == 1) {
		focal_init_success = 1;
		if (g_ASUS_hwID >= ZE500KL_SR1 )
			FOCAL_IRQ_DISABLE = false;
	}

	ftxxxx_irq_enable(ftxxxx_ts->client);
*/
	/* ---move to delay workque--- */
	return 0;
exit_err_sdev_register_fail:
exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ftxxxx_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ftxxxx_ts->pdata->reset);
#endif
#endif

err_create_wq_failed:
	if (ftxxxx_ts->init_check_ic_wq) {
		destroy_workqueue(ftxxxx_ts->init_check_ic_wq);
	}
	if (ftxxxx_ts->reset_wq) {
		destroy_workqueue(ftxxxx_ts->reset_wq);
	}
#ifdef FTS_PM
	if (ftxxxx_ts->suspend_resume_wq) {
		destroy_workqueue(ftxxxx_ts->suspend_resume_wq);
	}
#endif
	if (ftxxxx_ts->usb_wq) {
		destroy_workqueue(ftxxxx_ts->usb_wq);
	}
exit_irq_request_failed:
	fts_power_on(ftxxxx_ts, true);
exit_power_init:
	fts_power_init(&client->dev, ftxxxx_ts->pdata, false);
exit_init_gpio:
	fts_un_init_gpio_hw(ftxxxx_ts);
/*exit_init_gpio_TP0ID:
	fts_un_init_gpio_TP0ID(ftxxxx_ts);
exit_init_gpio_TP1ID:
	fts_un_init_gpio_TP1ID(ftxxxx_ts);
//exit_init_gpio_LCD0ID:
	fts_un_init_gpio_LCD0ID(ftxxxx_ts);
//exit_init_gpio_LCD1ID:
	fts_un_init_gpio_LCD1ID(ftxxxx_ts);*/
	i2c_set_clientdata(client, NULL);
	kfree(ftxxxx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
#ifdef FTS_PM
void ftxxxx_ts_suspend(void)
{
	g_bl_suspend = true;
	queue_work(ftxxxx_ts->suspend_resume_wq, &ftxxxx_ts->suspend_work);
	return;
}
EXPORT_SYMBOL(ftxxxx_ts_suspend);

void ftxxxx_ts_resume(void)
{
	if (focal_init_success==1) {
		g_bl_suspend = false;
	queue_work(ftxxxx_ts->suspend_resume_wq, &ftxxxx_ts->resume_work);
	}
	return;
}
EXPORT_SYMBOL(ftxxxx_ts_resume);
#endif
void ftxxxx_ts_shutdown(struct i2c_client *client)
{	
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 0);
	printk("[touch] reset_gpio =0\n");
	
}
static int ftxxxx_ts_remove(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	input_unregister_device(ftxxxx_ts->input_dev);
	/*gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 0);
	printk("[touch] reset_gpio =0");*/
#ifdef CONFIG_PM
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	ftxxxx_remove_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_release_apk_debug_channel();
#endif

	fts_un_init_gpio_hw(ftxxxx_ts);

	free_irq(client->irq, ftxxxx_ts);

	kfree(ftxxxx_ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ftxxxx_ts_id[] = {
	{ FTXXXX_NAME, 0 },
	{ }
};

//#ifdef CONFIG_OF
static struct of_device_id ft5x46_match_table[] = {
	{ .compatible = "focaltech,5X46",},
	{ },
};
/*#else
#define ft5x46_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, ftxxxx_ts_id);*/

static struct i2c_driver ftxxxx_ts_driver = {
	.probe = ftxxxx_ts_probe,
	.remove = ftxxxx_ts_remove,
 	.shutdown =ftxxxx_ts_shutdown,
	.id_table = ftxxxx_ts_id,
	.driver = {
		.name = FTXXXX_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ft5x46_match_table,
	},
};

static int __init ftxxxx_ts_init(void)
{
	int ret;
	printk("[Focal][Touch] %s : ftxxxx_ts_init !\n", __func__);
	ret = i2c_add_driver(&ftxxxx_ts_driver);
	if (ret) {
		printk(KERN_WARNING " [Focal][TOUCH_ERR] Adding ftxxxx driver failed "
			"(errno = %d)\n", ret);
	} else {
		printk("[Focal][Touch] %s : Successfully added driver %s\n", __func__,
			ftxxxx_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ftxxxx_ts_exit(void)
{
	i2c_del_driver(&ftxxxx_ts_driver);
}

module_init(ftxxxx_ts_init);
module_exit(ftxxxx_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
