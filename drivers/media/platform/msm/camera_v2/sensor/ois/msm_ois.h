/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_OIS_H
#define MSM_OIS_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define	MSM_OIS_MAX_VREGS (10)

struct msm_ois_ctrl_t;

enum msm_ois_state_t {
	OIS_ENABLE_STATE,
	OIS_OPS_ACTIVE,
	OIS_OPS_INACTIVE,
	OIS_DISABLE_STATE,
};

struct msm_ois_vreg {
	struct camera_vreg_t *cam_vreg;
	void *data[MSM_OIS_MAX_VREGS];
	int num_vreg;
};

/*ASUS_BSP +++ bill_chen "Implement ois"*/
struct msm_ois_board_info {
	char ois_name[MAX_OIS_NAME_SIZE];
	uint32_t i2c_slaveaddr;
	struct msm_camera_power_ctrl_t power_info;
	enum i2c_freq_mode_t i2c_freq_mode;
	struct msm_ois_opcode opcode;
};
/*ASUS_BSP --- bill_chen "Implement ois"*/

struct msm_ois_ctrl_t {
	struct i2c_driver *i2c_driver;
	struct platform_driver *pdriver;
	struct platform_device *pdev;
	struct msm_camera_i2c_client i2c_client;
	enum msm_camera_device_type_t ois_device_type;
	struct msm_sd_subdev msm_sd;
	struct mutex *ois_mutex;
	enum msm_camera_i2c_data_type i2c_data_type;
	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *ois_v4l2_subdev_ops;
	void *user_data;
	uint16_t i2c_tbl_index;
	enum cci_i2c_master_t cci_master;
	uint32_t subdev_id;
	enum msm_ois_state_t ois_state;
	struct msm_ois_vreg vreg_cfg;
	struct msm_camera_gpio_conf *gconf;
	struct msm_pinctrl_info pinctrl_info;
	uint8_t cam_pinctrl_status;
	/*ASUS_BSP +++ bill_chen "Implement ois"*/
	struct msm_ois_board_info *oboard_info;
	int32_t userspace_probe;
	//enum i2c_freq_mode_t i2c_freq_mode;
	uint32_t sensor_id_reg_addr;
	uint32_t sensor_id;
	/*ASUS_BSP --- bill_chen "Implement ois"*/
};

/*ASUS_BSP +++ bill_chen "Implement ois"*/
#define OIS_READ_CALIBRATION_COUNT 16
#define OIS_READ_CALIBRATION_SHIFT 4
#define PROGRAM_DOWNLOAD_OIS_OP_CODE 0x80
#define COEFFICIENT_DOWNLOAD_OIS_OP_CODE 0x88
#define PROGRAM_DOWNLOAD_TRNS_SIZE 32

int PROGRAM_DOWNLOAD_OIS_FW[5000];
int COEFFICIENT_DOWNLOAD_OIS_FW[1000];

int PROGRAM_DOWNLOAD_OIS_FW_LENGTH;
int COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH;

uint16_t OIS_CALIBRATION_GRYO_OFFSET_X[OIS_READ_CALIBRATION_COUNT + 1] = {0x0};
uint16_t OIS_CALIBRATION_GRYO_OFFSET_Y[OIS_READ_CALIBRATION_COUNT + 1] = {0x0};
uint16_t debug_calibration_data[6000] = {0x0};
/*{write ois address from eeprom, value}*/
uint16_t OIS_EEPROM_WRITE_ADDR_VALUE[] = {
   /*HALL*/
   0x8230,
   0x8231,
   0x8232,
   0x841E,
   0x849E,
   0x8239,
   0x823B,
   /*GYRO OFFSET*/
   0x8406,
   0x8486,
   /*Hall sensitivity*/
   0x8446,
   0x84C6,
   /*Loop gain*/
   0x840F,
   0x848F,
   /*Hall shift*/
   0x8231,
   0x8232,
   /*Temparature*/
   0x846A,
   0x846B,
   /*Kgx*/
   0x8470,
   0x8472
};

/*{eeprom address, value}*/
/*{eeprom address, value} */
uint16_t OIS_EEPROM_READ_ADDR_VALUE_DEFAULT[][2] = {
  {0x21B, 0x01d3}, {0x21D, 0x01fe}, {0x21F, 0x01e5}, {0x221, 0x0009}, {0x223, 0x00a9},
  {0x225, 0x007f}, {0x227, 0x007e}, {0x229, 0xffeb}, {0x22B, 0xfff2}, {0x22D, 0x2192},
  {0x22F, 0x2134}, {0x231, 0x1f48}, {0x233, 0x1fc6}, {0x235, 0x01fe}, {0x237, 0x01e5},
  {0x239, 0x0000}, {0x23B, 0x0000}, {0x23D, 0xff33}, {0x23F, 0xffaf},
};

/*{eeprom address, value}*/
uint16_t OIS_EEPROM_READ_ADDR_VALUE[][2] = {
  {0x21B, 0xffff}, {0x21D, 0xffff}, {0x21F, 0xffff}, {0x221, 0xffff}, {0x223, 0xffff},
  {0x225, 0xffff}, {0x227, 0xffff}, {0x229, 0xffff}, {0x22B, 0xffff}, {0x22D, 0xffff},
  {0x22F, 0xffff}, {0x231, 0xffff}, {0x233, 0xffff}, {0x235, 0xffff}, {0x237, 0xffff},
  {0x239, 0xffff}, {0x23B, 0xffff}, {0x23D, 0xffff}, {0x23F, 0xffff},
};

uint16_t OIS_MODE_READ_ADDR_VALUE[][6] = {{0x847F, 0x0c0c, 0x0d0d, 0x0d0d, 0x0d0d, 0}, /*{0x841B, 0x0000, 0x0001, 0x0001, 0x8000, 0}*/};

uint16_t OIS_GYRO_ACC_VALUE[] = {
	0x8455,  /*Gyro X*/
	0x8456,  /*Gyro Y*/
	0x8280,  /*ACC X*/
	0x82C0   /*ACC Y*/
};

/* Read many words(two bytes one time) from file */
static int Sysfs_read_word_seq(char *filename, int *value, int size);
/* Write many words(two bytes one time) to file */
static bool Sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size);
/* Read byte from file */
static int Sysfs_read_byte_seq(char *filename, int *value, int size);
static int Sysfs_read_char_seq(char *filename, int *value, int size);
static bool Sysfs_write_debug_seq(char *filename, uint16_t *value, uint32_t size);

static struct reg_settings_ois_t ois_read_calibration_setting_array[] = {
	/*gyro offset x = 0x8455*/
	{.reg_addr = 0x8455, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0001, .data_type = MSM_CAMERA_I2C_READ_CALIBRATION_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},

	/*gyro offset y = 0x8456*/
	{.reg_addr = 0x8456, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0002, .data_type = MSM_CAMERA_I2C_READ_CALIBRATION_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},
}; //end pantilt_on_ois_setting_size


/*Normal flow =>
  1. init_setting
  2. write_eeprom_data_setting
  3. write_calbration_data_setting
  4. ois_dsp_start_setting
*/

static struct reg_settings_ois_t ois_init_setting_array[] = {
	/*PLL Setting_0 8*/  
	{.reg_addr = 0x8262, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF02, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8263, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x9F05, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},
    /*
    clock:
    24000 => DIV_N = 0xBF03
    19200 => DIV_N = 0xFF02  V

    reference clock:
    36000 => DIV_M = 0x9F05  V
	27000 => DIV_M = 0x3704
	*/
	{.reg_addr = 0x8264, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x6040, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

	{.reg_addr = 0x8260, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x1130, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

	{.reg_addr = 0x8265, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

	{.reg_addr = 0x8261, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0280, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

	{.reg_addr = 0x8261, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0380, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

	{.reg_addr = 0x8261, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0988, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},																

    //program download 
	{.reg_addr = 0x8200, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0001, .data_type = MSM_CAMERA_I2C_READ_FW_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},

	{.reg_addr = 0x8200, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0001, .data_type = MSM_CAMERA_I2C_WRITE_FW_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},

    /*program download status check*/
	{.reg_addr = 0x84F7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},
    /*program download fw version check*/	
	{.reg_addr = 0x84F6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},

    //coefficient download 
	{.reg_addr = 0x8200, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0002, .data_type = MSM_CAMERA_I2C_READ_FW_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},

	{.reg_addr = 0x8200, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0002, .data_type = MSM_CAMERA_I2C_WRITE_FW_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},

	/*Coefficient Download fw version check*/
	{.reg_addr = 0x8476, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},
};


static struct reg_settings_ois_t ois_write_eeprom_data_setting_array[] = {
	/*read calibration data from eeprom data*/
	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_READ_EEPROM_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},
	/*write eeprom data to register*/
	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WRITE_EEPROM_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static struct reg_settings_ois_t ois_write_calibration_setting_array[] = {
	/*write calibration gyro offset X value from factory data*/
	{.reg_addr = 0x8406, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0001, .data_type = MSM_CAMERA_I2C_WRITE_CALIBRATION_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},
	/*read calibration  gyro offset X value from factory data*/
	//{.reg_addr = 0x8406, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},
	/*write calibration gyro offset Y value from factory data*/
	{.reg_addr = 0x8486, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0002, .data_type = MSM_CAMERA_I2C_WRITE_CALIBRATION_DATA,.i2c_operation = MSM_OIS_WRITE, .delay = 0},
	/*read calibration  gyro offset Y value from factory data*/
	//{.reg_addr = 0x8486, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFFFF, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},
};

static struct reg_settings_ois_t ois_dsp_start_setting_array[] = {
	/*DSP calculation START*/
	{.reg_addr = 0x8c01, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x01, .data_type = MSM_CAMERA_I2C_NO_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 10},

	//check ois status
	{.reg_addr = 0x8200, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0e75, .data_type = MSM_CAMERA_I2C_WORD_DATA,.i2c_operation = MSM_OIS_READ, .delay = 0},
};

static struct reg_settings_ois_t ois_enable_setting_array[] = {
	/*enable ois*/
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static struct reg_settings_ois_t ois_disable_setting_array[] = {
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0000, .data_type = MSM_CAMERA_I2C_READ_MODE_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},
};

static struct reg_settings_ois_t ois_movie_setting_array[] = {
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFD7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFD7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x2B32, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x2B32, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0001, .data_type = MSM_CAMERA_I2C_READ_MODE_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0}, 
};

static struct reg_settings_ois_t ois_still_setting_array[] = {
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFD7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFD7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x2B32, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x2B32, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0002, .data_type = MSM_CAMERA_I2C_READ_MODE_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},
};

static struct reg_settings_ois_t ois_test_setting_array[] = {
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x5909, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x5909, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0746, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0746, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},

	{.reg_addr = 0xFFFF, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, .reg_data = 0x0003, .data_type = MSM_CAMERA_I2C_READ_MODE_DATA, .i2c_operation = MSM_OIS_READ, .delay = 0},
};

/*ASUS_BSP --- bill_chen "Implement ois"*/
#endif
