/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/firmware.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"
/*ASUS_BSP +++ bill_chen "Implement ois"*/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/clk.h>
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"
/*ASUS_BSP --- bill_chen "Implement ois"*/

DEFINE_MSM_MUTEX(msm_ois_mutex);
/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct v4l2_file_operations msm_ois_v4l2_subdev_fops;
static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl);
static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl);

static struct i2c_driver msm_ois_i2c_driver;

/*ASUS_BSP +++ bill_chen "Implement ois"*/
static int32_t msm_ois_check_id(struct msm_ois_ctrl_t *o_ctrl);
static int32_t msm_ois_check_vm(struct msm_ois_ctrl_t *o_ctrl);
#if 0
static struct msm_cam_clk_info cam_8953_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 19200000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};
#endif
static int32_t msm_ois_get_dt_data(struct msm_ois_ctrl_t *o_ctrl); 
static struct msm_ois_ctrl_t *msm_ois_t_pointer = NULL;
/*ASUS_BSP --- bill_chen "Implement ois"*/

/*ASUS_BSP +++ bill_chen "Implement camera ois"*/ 
static unsigned char g_ois_status = 0;
static unsigned char g_ois_status_created = 0;
static void create_ois_status_proc_file(void);

static unsigned char g_ois_vm_created = 0; 
static void create_ois_vm_proc_file(void);


static unsigned char g_ois_cali_created = 0;  
static void create_ois_cali_proc_file(void); 

static unsigned char g_ois_mode = 255;
static unsigned char g_ois_mode_created = 0;
static void create_ois_mode_proc_file(void);

static int g_reg, g_value;
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
static unsigned char g_ois_rw_created = 0;
static void create_ois_rw_proc_file(void);

static int g_power_state = -1;
static unsigned char g_ois_power_created = 0;
static void create_ois_power_proc_file(void);

static int g_ois_dac = 0;
static unsigned char g_ois_dac_created = 0;
static void create_ois_dac_proc_file(void);

static unsigned char g_ois_debug_status = 0;
static unsigned char g_ois_debug_created = 0;
static void create_ois_debug_proc_file(void);

char ois_subdev_string[32] = "";
static unsigned char g_ois_device_created = 0;
static void create_ois_device_proc_file(void);
/*ASUS_BSP --- bill_chen "Implement camera ois"*/
static int32_t msm_ois_download(struct msm_ois_ctrl_t *o_ctrl)
{
	uint16_t bytes_in_tx = 0;
	uint16_t total_bytes = 0;
	uint8_t *ptr = NULL;
	int32_t rc = 0;
	const struct firmware *fw = NULL;
	const char *fw_name_prog = NULL;
	const char *fw_name_coeff = NULL;
	char name_prog[MAX_SENSOR_NAME] = {0};
	char name_coeff[MAX_SENSOR_NAME] = {0};
	struct device *dev = &(o_ctrl->pdev->dev);
	enum msm_camera_i2c_reg_addr_type save_addr_type;

	CDBG("Enter\n");
	save_addr_type = o_ctrl->i2c_client.addr_type;
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	snprintf(name_coeff, MAX_SENSOR_NAME, "%s.coeff",
		o_ctrl->oboard_info->ois_name);

	snprintf(name_prog, MAX_SENSOR_NAME, "%s.prog",
		o_ctrl->oboard_info->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		dev_err(dev, "Failed to locate %s\n", fw_name_prog);
		o_ctrl->i2c_client.addr_type = save_addr_type;
		return rc;
	}

	total_bytes = fw->size;
	for (ptr = (uint8_t *)fw->data; total_bytes;
		total_bytes -= bytes_in_tx, ptr += bytes_in_tx) {
		bytes_in_tx = (total_bytes > 10) ? 10 : total_bytes;
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&o_ctrl->i2c_client, o_ctrl->oboard_info->opcode.prog,
			 ptr, bytes_in_tx);
		if (rc < 0) {
			pr_err("Failed: remaining bytes to be downloaded: %d",
				bytes_in_tx);
			/* abort download fw and return error*/
			goto release_firmware;
		}
	}
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		dev_err(dev, "Failed to locate %s\n", fw_name_coeff);
		o_ctrl->i2c_client.addr_type = save_addr_type;
		return rc;
	}
	total_bytes = fw->size;
	for (ptr = (uint8_t *)fw->data; total_bytes;
		total_bytes -= bytes_in_tx, ptr += bytes_in_tx) {
		bytes_in_tx = (total_bytes > 10) ? 10 : total_bytes;
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&o_ctrl->i2c_client, o_ctrl->oboard_info->opcode.coeff,
			ptr, bytes_in_tx);
		if (rc < 0) {
			pr_err("Failed: remaining bytes to be downloaded: %d",
				total_bytes);
			/* abort download fw*/
			break;
		}
	}
release_firmware:
	release_firmware(fw);
	o_ctrl->i2c_client.addr_type = save_addr_type;

	return rc;
}

static int32_t msm_ois_data_config(struct msm_ois_ctrl_t *o_ctrl,
	struct msm_ois_slave_info *slave_info)
{
	int rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;

	CDBG("Enter\n");
	if (!slave_info) {
		pr_err("failed : invalid slave_info ");
		return -EINVAL;
	}
	/* fill ois slave info*/
	if (strlcpy(o_ctrl->oboard_info->ois_name, slave_info->ois_name,
		sizeof(o_ctrl->oboard_info->ois_name)) < 0) {
		pr_err("failed: copy_from_user");
		return -EFAULT;
	}
	memcpy(&(o_ctrl->oboard_info->opcode), &(slave_info->opcode),
		sizeof(struct msm_ois_opcode));
	o_ctrl->oboard_info->i2c_slaveaddr = slave_info->i2c_addr;

	/* config cci_client*/
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = o_ctrl->i2c_client.cci_client;
		cci_client->sid =
			o_ctrl->oboard_info->i2c_slaveaddr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = o_ctrl->cci_master;
	} else {
		o_ctrl->i2c_client.client->addr =
			o_ctrl->oboard_info->i2c_slaveaddr;
	}
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_write_settings(struct msm_ois_ctrl_t *o_ctrl,
	uint16_t size, struct reg_settings_ois_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	struct msm_camera_i2c_seq_reg_array *reg_setting;
	/*ASUS_BSP +++ bill_chen "Implement ois"*/
	int16_t j = 0;
	uint16_t read_num = 0;
	uint16_t block_cnt = 0;
	uint16_t total_cnt = 0;
	uint16_t register_array_size = 0;
	uint16_t fw_array_size = 0;
	int read_file_num[2] = {0, 0};
	uint16_t write_file_num[6] = {0};
	bool file_rc = false;
	int32_t sum = 0;
	int16_t max_gyro_x = 0x8000;
	int16_t max_gyro_y = 0x8000;
	int16_t min_gyro_x = 0x7FFF;
	int16_t min_gyro_y = 0x7FFF;
	uint16_t save_sid = 0;
	/*ASUS_BSP --- bill_chen "Implement ois"*/
	pr_err("Randy ois Enter writing setting, size=%d, setting[0].address=0x%X\n", size, settings[0].reg_addr);

	for (i = 0; i < size; i++) {
		switch (settings[i].i2c_operation) {
		case MSM_OIS_WRITE: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&o_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data,
					settings[i].data_type);
				break;
			case MSM_CAMERA_I2C_DWORD_DATA:
			reg_setting =
			kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;

				reg_setting->reg_addr = settings[i].reg_addr;
				reg_setting->reg_data[0] = (uint8_t)
					((settings[i].reg_data &
					0xFF000000) >> 24);
				reg_setting->reg_data[1] = (uint8_t)
					((settings[i].reg_data &
					0x00FF0000) >> 16);
				reg_setting->reg_data[2] = (uint8_t)
					((settings[i].reg_data &
					0x0000FF00) >> 8);
				reg_setting->reg_data[3] = (uint8_t)
					(settings[i].reg_data & 0x000000FF);
				reg_setting->reg_data_size = 4;
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_write_seq(&o_ctrl->i2c_client,
					reg_setting->reg_addr,
					reg_setting->reg_data,
					reg_setting->reg_data_size);
				kfree(reg_setting);
				reg_setting = NULL;
				if (rc < 0)
					return rc;
				break;

			/*ASUS_BSP +++ bill_chen "Implement ois"*/
			case MSM_CAMERA_I2C_WRITE_EEPROM_DATA:
				pr_err("%s: write ois eeprom", __func__);
				for(j = 0; j < ARRAY_SIZE(OIS_EEPROM_WRITE_ADDR_VALUE); j++) {
					if(OIS_EEPROM_READ_ADDR_VALUE[0][1] != 0xffff && OIS_EEPROM_READ_ADDR_VALUE[1][1] != 0xffff) {
						rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
							&o_ctrl->i2c_client,
							OIS_EEPROM_WRITE_ADDR_VALUE[j],
							OIS_EEPROM_READ_ADDR_VALUE[j][1],
							MSM_CAMERA_I2C_WORD_DATA);
						//pr_err("eeprom[0x%x]=0x%x ", OIS_EEPROM_WRITE_ADDR_VALUE[j], OIS_EEPROM_READ_ADDR_VALUE[j][1]);
					} else {
						rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
							&o_ctrl->i2c_client,
							OIS_EEPROM_WRITE_ADDR_VALUE[j],
							OIS_EEPROM_READ_ADDR_VALUE_DEFAULT[j][1],
							MSM_CAMERA_I2C_WORD_DATA);
						//pr_err("default_eeprom[0x%x]=0x%x ", OIS_EEPROM_WRITE_ADDR_VALUE[j], OIS_EEPROM_READ_ADDR_VALUE_DEFAULT[j][1]);
					}
					if (rc < 0) {
						pr_err("%s: write ois eeprom i2c_write fail rc = %d\n", __func__, rc);
						return rc;
					}
				}
				rc = 0;
				break;
			case MSM_CAMERA_I2C_WRITE_CALIBRATION_DATA:
				reg_setting =
				kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;
				//pr_err("%s: MSM_CAMERA_I2C_WRITE_CALIBRATION_DATA +++\n", __func__);
				reg_setting->reg_addr = settings[i].reg_addr;
				file_rc = Sysfs_read_word_seq("/factory/OIS_calibration", read_file_num, 2);
				//pr_err("%s: WRITE_CALIBRATION_DATA read_num[0] = 0x%x, read_num[1] = 0x%x\n", __func__, read_file_num[0], read_file_num[1]);
				if(settings[i].reg_data == 0x0001 && file_rc == 0) {
					reg_setting->reg_data[0] = (uint8_t)
						(read_file_num[0] &
						0x00FF);
					reg_setting->reg_data[1] = (uint8_t)
						((read_file_num[0] &
						0xFF00) >> 8);
				} else if(settings[i].reg_data == 0x0002 && file_rc == 0) {
					reg_setting->reg_data[0] = (uint8_t)
						(read_file_num[1] &
						0x00FF);
					reg_setting->reg_data[1] = (uint8_t)
						((read_file_num[1] &
						0xFF00) >> 8);
				} else {
					rc = 0;
					kfree(reg_setting);
					reg_setting = NULL;
					break;
				}

				pr_err("%s: WRITE_CALIBRATION_DATA reg_setting.reg_data[0] = 0x%x, reg_data[1] = 0x%x\n", __func__, reg_setting->reg_data[0], reg_setting->reg_data[1]);
				reg_setting->reg_data_size = 2;
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_write_seq(&o_ctrl->i2c_client,
					reg_setting->reg_addr,
					reg_setting->reg_data,
					reg_setting->reg_data_size);
				kfree(reg_setting);
				reg_setting = NULL;
				if (rc < 0)
					return rc;
				//pr_err("%s: MSM_CAMERA_I2C_WRITE_CALIBRATION_DATA ---\n", __func__);
				break;
			case MSM_CAMERA_I2C_NO_DATA:
				reg_setting =
				kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;
				o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
				reg_setting->reg_addr = (settings[i].reg_addr & 0xFF00) >> 8;
				reg_setting->reg_data[0] = (uint8_t)
					(settings[i].reg_addr &
					0x00FF);
				reg_setting->reg_data_size = 1;
				pr_err("%s: NODATA write[0x%x] = 0x%x\n", __func__, reg_setting->reg_addr, reg_setting->reg_data[0]);
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_write_seq(&o_ctrl->i2c_client,
					reg_setting->reg_addr,
					reg_setting->reg_data,
					reg_setting->reg_data_size);
				o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
				kfree(reg_setting);
				reg_setting = NULL;
				if (rc < 0)
					return rc;
				break;
			case MSM_CAMERA_I2C_WRITE_FW_DATA:
				reg_setting =
				kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;
				//pr_err("%s: MSM_CAMERA_I2C_WRITE_FW_DATA +++\n", __func__);
				if(settings[i].reg_data == 0x0001) {
				   fw_array_size = PROGRAM_DOWNLOAD_OIS_FW_LENGTH;
				} else if(settings[i].reg_data == 0x0002) {
				   fw_array_size = COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH;
				}
				block_cnt = fw_array_size / PROGRAM_DOWNLOAD_TRNS_SIZE + 1;
				total_cnt = block_cnt;
				pr_err("%s: fw_array_size = %d, block_cnt = %d, total_cnt = %d\n", __func__, fw_array_size, block_cnt, total_cnt);
				//usleep(50);
				while(block_cnt > 0) {
					if(block_cnt == 1) {
						register_array_size = fw_array_size % PROGRAM_DOWNLOAD_TRNS_SIZE;
					} else
						register_array_size = PROGRAM_DOWNLOAD_TRNS_SIZE;

					if(register_array_size != 0) {
						if(settings[i].reg_data == 0x0001) {
							reg_setting->reg_addr = PROGRAM_DOWNLOAD_OIS_OP_CODE << 8 |
								PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							for(j = 1; j < register_array_size; j++) {
								reg_setting->reg_data[j - 1] = (uint8_t)(0xFF & PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE + j]);
							}
						} else if(settings[i].reg_data == 0x0002) {
							reg_setting->reg_addr = COEFFICIENT_DOWNLOAD_OIS_OP_CODE << 8 |
								COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							for(j = 1; j < register_array_size; j++) {
								reg_setting->reg_data[j - 1] = (uint8_t)(0xFF & COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE + j]);
							}
						}
						reg_setting->reg_data_size = register_array_size - 1;
						if(register_array_size == 1) {
							o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
							if(settings[i].reg_data == 0x0001) {
							    reg_setting->reg_addr = PROGRAM_DOWNLOAD_OIS_OP_CODE;
								reg_setting->reg_data[0] = PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							} else if(settings[i].reg_data == 0x0002) {
							    reg_setting->reg_addr = COEFFICIENT_DOWNLOAD_OIS_OP_CODE;
								reg_setting->reg_data[0] = COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							}
							reg_setting->reg_data_size = 1;
						}
                                            usleep_range(1,2);  //ASUS_BSP Deka ''fix startpreiew and OIS init compete i2c resource issue"
						if(settings[i].reg_data == 0x0001 || settings[i].reg_data == 0x0002) {
							rc = o_ctrl->i2c_client.i2c_func_tbl->
								i2c_write_seq(&o_ctrl->i2c_client,
								reg_setting->reg_addr,
								reg_setting->reg_data,
								reg_setting->reg_data_size);
							if (rc < 0) {
								if(register_array_size == 1) {
									o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
								}
								pr_err("%s: MSM_CAMERA_I2C_WRITE_FW_DATA block_cnt = %d rc = %d fail ---\n", __func__, block_cnt, rc);
								kfree(reg_setting);
								reg_setting = NULL;
								return rc;
							}
						} else {
							rc = 0;
						}
						if(register_array_size == 1) {
						    o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
						}
					}
					block_cnt--;
				}
				kfree(reg_setting);
				reg_setting = NULL;
				//pr_err("%s: MSM_CAMERA_I2C_WRITE_FW_DATA ---\n", __func__);
				break;
			/*ASUS_BSP --- bill_chen "Implement ois"*/

			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
		}
			break;

	    /*ASUS_BSP +++ bill_chen "Implement ois"*/
 		case MSM_OIS_READ: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_READ_EEPROM_DATA:
				save_sid = o_ctrl->i2c_client.cci_client->sid;
				o_ctrl->i2c_client.cci_client->sid = 0x50; //EEPROM
				pr_err("1 %s: OIS_EEPROM_READ_ADDR_VALUE = %ld\n", __func__, ARRAY_SIZE(OIS_EEPROM_READ_ADDR_VALUE));
				for(j = 0; j < ARRAY_SIZE(OIS_EEPROM_READ_ADDR_VALUE); j++) {
					rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
						&o_ctrl->i2c_client,
						OIS_EEPROM_READ_ADDR_VALUE[j][0],
						&(OIS_EEPROM_READ_ADDR_VALUE[j][1]),
						MSM_CAMERA_I2C_WORD_DATA);
					//pr_err("%s: eeprom value [0x%x] = 0x%x ", __func__, OIS_EEPROM_READ_ADDR_VALUE[j][0], OIS_EEPROM_READ_ADDR_VALUE[j][1]);
					if (rc < 0) {
					   pr_err("%s: read fail rc = %d\n", __func__, rc);
					   o_ctrl->i2c_client.cci_client->sid = save_sid;
					   return rc;
					}
				}
				o_ctrl->i2c_client.cci_client->sid = save_sid;
				break;
			case MSM_CAMERA_I2C_READ_MODE_DATA:
				//pr_err("%s: MSM_CAMERA_I2C_READ_MODE_DATA settings[i].reg_data = 0x%x\n", __func__, settings[i].reg_data);
				//pr_err("%s: MSM_CAMERA_I2C_READ_MODE_DATA OIS_MODE_READ_ADDR_VALUE size = %ld\n", __func__, ARRAY_SIZE(OIS_MODE_READ_ADDR_VALUE));
				for(j = 0; j < ARRAY_SIZE(OIS_MODE_READ_ADDR_VALUE); j++) {
					rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
						&o_ctrl->i2c_client,
						OIS_MODE_READ_ADDR_VALUE[j][0],
						&read_num,
						MSM_CAMERA_I2C_WORD_DATA);

					//pr_err("%s: find ois mode value[%d] = 0x%x read num = 0x%x\n", __func__, j, OIS_MODE_READ_ADDR_VALUE[j][0], read_num);
					//pr_err("%s: expected number[1] = 0x%x, number[2] = 0x%x, number[3] = 0x%x\n, number[4] = 0x%x", __func__, OIS_MODE_READ_ADDR_VALUE[j][1], OIS_MODE_READ_ADDR_VALUE[j][2], OIS_MODE_READ_ADDR_VALUE[j][3], OIS_MODE_READ_ADDR_VALUE[j][4]);
					OIS_MODE_READ_ADDR_VALUE[j][5] = ((read_num & 0xFF00) >> 8) + ((read_num & 0x00FF) << 8);
					if(rc < 0) {
						pr_err("%s: MSM_CAMERA_I2C_READ_MODE_DATA i2c read fail rc = %d, settings[i].reg_data = 0x%x\n", __func__, rc, settings[i].reg_data);
						return rc;
					}
				}

				for(j = 0; j < ARRAY_SIZE(OIS_MODE_READ_ADDR_VALUE); j++) {
					//pr_err("%s: OIS_MODE_READ_ADDR_VALUE[%d][5] = 0x%x, OIS_MODE_READ_ADDR_VALUE[%d][%d] = 0x%x\n", __func__, j, OIS_MODE_READ_ADDR_VALUE[j][5], j, settings[i].reg_data + 1, OIS_MODE_READ_ADDR_VALUE[j][settings[i].reg_data + 1]);
					if(OIS_MODE_READ_ADDR_VALUE[j][5] != OIS_MODE_READ_ADDR_VALUE[j][settings[i].reg_data + 1]) {
						break;
					}
				}

				if(j == 1 && settings[i].reg_data == 0x0000) {
					g_ois_mode = settings[i].reg_data;
				} else if(j == ARRAY_SIZE(OIS_MODE_READ_ADDR_VALUE)) {
					g_ois_mode = settings[i].reg_data;
				} else{
					g_ois_mode = 255;
				}
				pr_err("%s: now_ois_mode = %d, expected_ois_mode = 0x%x\n", __func__, g_ois_mode, settings[i].reg_data);
				//pr_err("%s: MSM_CAMERA_I2C_READ_MODE_DATA ---\n", __func__);
				break;
			case MSM_CAMERA_I2C_READ_FW_DATA:
				//pr_err("%s: MSM_CAMERA_I2C_READ_FW_DATA +++\n", __func__);
				if(settings[i].reg_data == 0x0001) {
					PROGRAM_DOWNLOAD_OIS_FW_LENGTH = Sysfs_read_byte_seq("/asusfw/ois/OIS_programFW.bin", PROGRAM_DOWNLOAD_OIS_FW, ARRAY_SIZE(PROGRAM_DOWNLOAD_OIS_FW));
					pr_err("%s: PROGRAM_DOWNLOAD_OIS_FW_LENGTH = 0x%d\n", __func__, PROGRAM_DOWNLOAD_OIS_FW_LENGTH);
				} else if(settings[i].reg_data == 0x0002) {
					COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH = Sysfs_read_byte_seq("/asusfw/ois/OIS_coefficientFW.mem", COEFFICIENT_DOWNLOAD_OIS_FW, ARRAY_SIZE(COEFFICIENT_DOWNLOAD_OIS_FW));
					pr_err("%s: COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH = 0x%d\n", __func__, COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH);
				}
				//pr_err("%s: MSM_CAMERA_I2C_READ_FW_DATA ---\n", __func__);
				rc = 0;
				break;
			case MSM_CAMERA_I2C_READ_CALIBRATION_DATA:
				//pr_err("%s: MSM_CAMERA_I2C_READ_CALIBRATION_DATA +++\n", __func__);
				pr_err("%s: MSM_CAMERA_I2C_READ_CALIBRATION_DATA settings.reg_data = 0x%x, reg_addr = 0x%x\n", __func__, settings[i].reg_data, settings[i].reg_addr);
				sum = 0;
				usleep_range(10000, 11000);
				for(j = 0; j < OIS_READ_CALIBRATION_COUNT; j++) {
					rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
						&o_ctrl->i2c_client,
						settings[i].reg_addr,
						&read_num,
						MSM_CAMERA_I2C_WORD_DATA);
					sum += (int16_t)read_num;
					if(settings[i].reg_data == 0x0001)
						OIS_CALIBRATION_GRYO_OFFSET_X[j + 1] = read_num;
					else if(settings[i].reg_data == 0x0002)
						OIS_CALIBRATION_GRYO_OFFSET_Y[j + 1] = read_num;
					pr_err("%s: read calibration ois read_num[%d] = 0x%x(%d) sum = 0x%x(%d)\n", __func__, j, read_num, (int16_t)read_num, sum, sum);
					usleep_range(5000, 6000);
					if (rc < 0)
						return rc;
				}
				sum = sum >> OIS_READ_CALIBRATION_SHIFT;
				pr_err("%s: read calibration ois sum >> 4 = 0x%x(%d)\n", __func__, sum, sum);

				if(settings[i].reg_data == 0x0001) {
					OIS_CALIBRATION_GRYO_OFFSET_X[0] = (uint16_t)(sum & 0xFFFF); //(uint16_t)(((sum & 0x00FF) << 8) + ((sum & 0xFF00) >> 8));
					pr_err("%s: OIS_CALIBRATION_GRYO_OFFSET_X[0] = 0x%x\n", __func__, OIS_CALIBRATION_GRYO_OFFSET_X[0]);
				} else if(settings[i].reg_data == 0x0002) {
					OIS_CALIBRATION_GRYO_OFFSET_Y[0] = (uint16_t)(sum & 0xFFFF);//(uint16_t)(((sum & 0x00FF) << 8) + ((sum & 0xFF00) >> 8));
					pr_err("%s: OIS_CALIBRATION_GRYO_OFFSET_Y[0] = 0x%x\n", __func__, OIS_CALIBRATION_GRYO_OFFSET_Y[0]);
					for(j = 1; j < OIS_READ_CALIBRATION_COUNT + 1; j++) {
						if(max_gyro_x < (int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[j])
							max_gyro_x = (int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[j];
						if(max_gyro_y < (int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[j])
							max_gyro_y = (int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[j];
						if(min_gyro_x > (int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[j])
							min_gyro_x = (int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[j];
						if(min_gyro_y > (int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[j])
							min_gyro_y = (int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[j];
					}
					write_file_num[0] = OIS_CALIBRATION_GRYO_OFFSET_X[0];
					write_file_num[1] = OIS_CALIBRATION_GRYO_OFFSET_Y[0];
					write_file_num[2] = (uint16_t)max_gyro_x;
					write_file_num[3] = (uint16_t)max_gyro_y;
					write_file_num[4] = (uint16_t)min_gyro_x;
					write_file_num[5] = (uint16_t)min_gyro_y;
					file_rc = Sysfs_write_word_seq("/factory/OIS_calibration", write_file_num, 6);
					pr_err("%s: file_rc = %d\n", __func__, file_rc);
				}

				//pr_err("%s: MSM_CAMERA_I2C_READ_CALIBRATION_DATA ---\n", __func__);
				break;
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&o_ctrl->i2c_client,
					settings[i].reg_addr,
					&read_num,
					settings[i].data_type);
				pr_err("%s: ois read[0x%x] = 0x%x\n", __func__, settings[i].reg_addr, read_num);
				if(settings[i].reg_data != 0xFFFF &&
					settings[i].reg_data != read_num) {
					pr_err("%s: ois read_num no match expected value = 0x%x\n", __func__, settings[i].reg_data);
				}
				break;
			case MSM_CAMERA_I2C_DWORD_DATA:
				reg_setting =
				kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;
				reg_setting->reg_addr = settings[i].reg_addr;
				reg_setting->reg_data[0] = (uint8_t)
					((settings[i].reg_data &
					0xFF000000) >> 24);
				reg_setting->reg_data[1] = (uint8_t)
					((settings[i].reg_data &
					0x00FF0000) >> 16);
				reg_setting->reg_data[2] = (uint8_t)
					((settings[i].reg_data &
					0x0000FF00) >> 8);
				reg_setting->reg_data[3] = (uint8_t)
					(settings[i].reg_data & 0x000000FF);
				reg_setting->reg_data_size = 4;
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_read_seq(&o_ctrl->i2c_client,
					reg_setting->reg_addr,
					reg_setting->reg_data,
					reg_setting->reg_data_size);
				kfree(reg_setting);
				reg_setting = NULL;
				if (rc < 0)
					return rc;
				break;
			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
			if (settings[i].delay > 20)
				msleep(settings[i].delay);
			else if (0 != settings[i].delay)
				usleep_range(settings[i].delay * 1000,
					(settings[i].delay * 1000) + 1000);
		}
			break;
		/*ASUS_BSP --- bill_chen "Implement ois"*/
		case MSM_OIS_POLL: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:

				rc = o_ctrl->i2c_client.i2c_func_tbl
					->i2c_poll(&o_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data,
					settings[i].data_type,
					settings[i].delay);
				break;
			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
		}
		}

		if (rc < 0)
			break;
	}

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_vreg_control(struct msm_ois_ctrl_t *o_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_ois_vreg *vreg_cfg;

	vreg_cfg = &o_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= MSM_OIS_MAX_VREGS) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(o_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}

static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	//enum msm_sensor_power_seq_gpio_t gpio;  /*ASUS_BSP bill_chen "Fix ois conflict"*/

	CDBG("Enter\n");
	pr_err("%s: E\n", __func__);  /*ASUS_BSP bill_chen "Implement ois"*/
	if (o_ctrl->ois_state != OIS_DISABLE_STATE) {

		rc = msm_ois_vreg_control(o_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

#if 0
		for (gpio = SENSOR_GPIO_AF_PWDM; gpio < SENSOR_GPIO_MAX;
			gpio++) {
			if (o_ctrl->gconf &&
				o_ctrl->gconf->gpio_num_info &&
				o_ctrl->gconf->
					gpio_num_info->valid[gpio] == 1) {
				gpio_set_value_cansleep(
					o_ctrl->gconf->gpio_num_info
						->gpio_num[gpio],
					GPIOF_OUT_INIT_LOW);

				if (o_ctrl->cam_pinctrl_status) {
					rc = pinctrl_select_state(
						o_ctrl->pinctrl_info.pinctrl,
						o_ctrl->pinctrl_info.
							gpio_state_suspend);
					if (rc < 0)
						pr_err("ERR:%s:%d cannot set pin to suspend state: %d",
							__func__, __LINE__, rc);
					devm_pinctrl_put(
						o_ctrl->pinctrl_info.pinctrl);
				}
				o_ctrl->cam_pinctrl_status = 0;
				rc = msm_camera_request_gpio_table(
					o_ctrl->gconf->cam_gpio_req_tbl,
					o_ctrl->gconf->cam_gpio_req_tbl_size,
					0);
				if (rc < 0)
					pr_err("ERR:%s:Failed in selecting state in ois power down: %d\n",
						__func__, rc);
			}
		}

#endif
		o_ctrl->i2c_tbl_index = 0;
		o_ctrl->ois_state = OIS_OPS_INACTIVE;
	}
	g_ois_mode = 255;
	pr_err("%s: X\n", __func__);  /*ASUS_BSP bill_chen "Implement ois"*/
	CDBG("Exit\n");
	return rc;
}

static int msm_ois_init(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");

	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	o_ctrl->ois_state = OIS_OPS_ACTIVE;
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_control(struct msm_ois_ctrl_t *o_ctrl,
	struct msm_ois_set_info_t *set_info)
{
	struct reg_settings_ois_t *settings = NULL;
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = o_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->ois_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = o_ctrl->cci_master;
		cci_client->i2c_freq_mode = set_info->ois_params.i2c_freq_mode;
	} else {
		o_ctrl->i2c_client.client->addr =
			set_info->ois_params.i2c_addr;
	}
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;


	if (set_info->ois_params.setting_size > 0 &&
		set_info->ois_params.setting_size
		< MAX_OIS_REG_SETTINGS) {
		settings = kmalloc(
			sizeof(struct reg_settings_ois_t) *
			(set_info->ois_params.setting_size),
			GFP_KERNEL);
		if (settings == NULL) {
			pr_err("Error allocating memory\n");
			return -EFAULT;
		}
		if (copy_from_user(settings,
			(void *)set_info->ois_params.settings,
			set_info->ois_params.setting_size *
			sizeof(struct reg_settings_ois_t))) {
			kfree(settings);
			pr_err("Error copying\n");
			return -EFAULT;
		}

		rc = msm_ois_write_settings(o_ctrl,
			set_info->ois_params.setting_size,
			settings);
		kfree(settings);
		if (rc < 0) {
			pr_err("Error\n");
			return -EFAULT;
		}
	}

	CDBG("Exit\n");

	return rc;
}


static int32_t msm_ois_config(struct msm_ois_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_ois_cfg_data *cdata =
		(struct msm_ois_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(o_ctrl->ois_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_OIS_INIT:
		rc = msm_ois_init(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_init failed %d\n", rc);
		break;
	case CFG_OIS_POWERDOWN:
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_power_down failed %d\n", rc);
		break;
	case CFG_OIS_POWERUP:
		rc = msm_ois_power_up(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois power up%d\n", rc);
		break;
	case CFG_OIS_CONTROL:
		rc = msm_ois_control(o_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("Failed ois control%d\n", rc);
		break;
	case CFG_OIS_I2C_WRITE_SEQ_TABLE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			memcpy(&conf_array,
				(void *)cdata->cfg.settings,
				sizeof(struct msm_camera_i2c_seq_reg_setting));
		} else
#endif
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.settings,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = o_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_seq_table(&o_ctrl->i2c_client,
			&conf_array);
		if(conf_array.reg_setting->reg_addr == 0xF090) {
			g_ois_dac = (((int)(conf_array.reg_setting->reg_data[1])) << 8) +
				        ((int)conf_array.reg_setting->reg_data[2]);
		}
		kfree(reg_setting);
		break;
	}
	/*ASUS_BSP +++ bill_chen "Implement ois command for dit 3A"*/
	case CFG_OIS_I2C_WRITE_MODE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			memcpy(&conf_array,
				(void *)cdata->cfg.settings,
				sizeof(struct msm_camera_i2c_seq_reg_setting));
		} else
#endif
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.settings,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if(conf_array.delay == g_ois_mode) { 
		    break;
		}

		if(conf_array.delay == 0) { 
			rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_disable_setting_array),
				ois_disable_setting_array);
		} else if(conf_array.delay == 1) {
			rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_enable_setting_array),
				ois_enable_setting_array);
			//rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_movie_setting_array),
				//ois_movie_setting_array);
			if(rc >= 0) {
				g_ois_mode = 1;
			}
		} else if(conf_array.delay == 2) {
			rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_enable_setting_array),
				ois_enable_setting_array);
			//rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_still_setting_array),
				//ois_still_setting_array);
			if(rc >= 0) {
				g_ois_mode = 2;
			}
		} else if(conf_array.delay == 3) {
			rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_enable_setting_array),
				ois_enable_setting_array);
			//rc = msm_ois_write_settings(msm_ois_t_pointer, ARRAY_SIZE(ois_test_setting_array),
				//ois_test_setting_array);
			if(rc >= 0) {
				g_ois_mode = 3;
			}
		}
		break;
	} 
	/*ASUS_BSP --- bill_chen "Implement ois command for dit 3A"*/
	default: 
		break;
	}
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_config_download(struct msm_ois_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_ois_cfg_download_data *cdata =
		(struct msm_ois_cfg_download_data *)argp;
	int32_t rc = 0;

	if (!o_ctrl || !cdata) {
		pr_err("failed: Invalid data\n");
		return -EINVAL;
	}
	mutex_lock(o_ctrl->ois_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_OIS_DATA_CONFIG:
		rc = msm_ois_data_config(o_ctrl, &cdata->slave_info);
		if (rc < 0)
			pr_err("Failed ois data config %d\n", rc);
		break;
	case CFG_OIS_DOWNLOAD:
		rc = msm_ois_download(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois download %d\n", rc);
		break;
	default:
		break;
	}
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}


static int32_t msm_ois_get_subdev_id(struct msm_ois_ctrl_t *o_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = o_ctrl->pdev->id;
	else
		*subdev_id = o_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll = msm_camera_qup_i2c_poll,
};

static int msm_ois_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_ois_ctrl_t *o_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	mutex_lock(o_ctrl->ois_mutex);
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE &&
		o_ctrl->ois_state != OIS_DISABLE_STATE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	o_ctrl->ois_state = OIS_DISABLE_STATE;
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_ois_internal_ops = {
	.close = msm_ois_close,
};

static long msm_ois_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int rc;
	struct msm_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d o_ctrl %pK argp %pK\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_ois_get_subdev_id(o_ctrl, argp);
	case VIDIOC_MSM_OIS_CFG:
		return msm_ois_config(o_ctrl, argp);
	case VIDIOC_MSM_OIS_CFG_DOWNLOAD:
		return msm_ois_config_download(o_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		if (!o_ctrl->i2c_client.i2c_func_tbl) {
			pr_err("o_ctrl->i2c_client.i2c_func_tbl NULL\n");
			return -EINVAL;
		}
		mutex_lock(o_ctrl->ois_mutex);
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0) {
			pr_err("%s:%d OIS Power down failed\n",
				__func__, __LINE__);
		}
		mutex_unlock(o_ctrl->ois_mutex);
		return msm_ois_close(sd, NULL);
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	//enum msm_sensor_power_seq_gpio_t gpio;  /*ASUS_BSP bill_chen "Fix ois conflict"*/

	CDBG("%s called\n", __func__);
	pr_err("%s: E\n", __func__);  /*ASUS_BSP bill_chen "Implement ois"*/

	rc = msm_ois_vreg_control(o_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	g_ois_mode = 255;  /*ASUS_BSP bill_chen "Implement ois"*/
#if 0
	for (gpio = SENSOR_GPIO_AF_PWDM;
		gpio < SENSOR_GPIO_MAX; gpio++) {
		if (o_ctrl->gconf && o_ctrl->gconf->gpio_num_info &&
			o_ctrl->gconf->gpio_num_info->valid[gpio] == 1) {
			rc = msm_camera_request_gpio_table(
				o_ctrl->gconf->cam_gpio_req_tbl,
				o_ctrl->gconf->cam_gpio_req_tbl_size, 1);
			if (rc < 0) {
				pr_err("ERR:%s:Failed in selecting state for ois: %d\n",
					__func__, rc);
				return rc;
			}
			if (o_ctrl->cam_pinctrl_status) {
				rc = pinctrl_select_state(
					o_ctrl->pinctrl_info.pinctrl,
					o_ctrl->pinctrl_info.gpio_state_active);
				if (rc < 0)
					pr_err("ERR:%s:%d cannot set pin to active state: %d",
						__func__, __LINE__, rc);
			}

			gpio_set_value_cansleep(
				o_ctrl->gconf->gpio_num_info->gpio_num[gpio],
				1);
		}
	}
#endif
	o_ctrl->ois_state = OIS_ENABLE_STATE;

	pr_err("%s: X\n", __func__);  /*ASUS_BSP bill_chen "Implement ois"*/
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_ois_subdev_core_ops = {
	.ioctl = msm_ois_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_ois_subdev_ops = {
	.core = &msm_ois_subdev_core_ops,
};

static const struct i2c_device_id msm_ois_i2c_id[] = {
	{"qcom,ois", (kernel_ulong_t)NULL},
	{ }
};
static int32_t msm_ois_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_ois_ctrl_t *ois_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_ois_i2c_probe: client is null\n");
		return -EINVAL;
	}

	ois_ctrl_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!ois_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	CDBG("client = 0x%pK\n",  client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&ois_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", ois_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto probe_failure;
	}

	ois_ctrl_t->i2c_driver = &msm_ois_i2c_driver;
	ois_ctrl_t->i2c_client.client = client;
	/* Set device type as I2C */
	ois_ctrl_t->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	ois_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	ois_ctrl_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
	ois_ctrl_t->ois_mutex = &msm_ois_mutex;

	/* Assign name for sub device */
	snprintf(ois_ctrl_t->msm_sd.sd.name, sizeof(ois_ctrl_t->msm_sd.sd.name),
		"%s", ois_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&ois_ctrl_t->msm_sd.sd,
		ois_ctrl_t->i2c_client.client,
		ois_ctrl_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&ois_ctrl_t->msm_sd.sd, ois_ctrl_t);
	ois_ctrl_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	ois_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&ois_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	ois_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	ois_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	ois_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&ois_ctrl_t->msm_sd);
	ois_ctrl_t->ois_state = OIS_DISABLE_STATE;
	pr_info("msm_ois_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	kfree(ois_ctrl_t);
	return rc;
}

#ifdef CONFIG_COMPAT
static long msm_ois_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	long rc = 0;
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct msm_ois_cfg_data32 *u32;
	struct msm_ois_cfg_data ois_data;
	void *parg;
	struct msm_camera_i2c_seq_reg_setting settings;
	struct msm_camera_i2c_seq_reg_setting32 settings32;

	if (!file || !arg) {
		pr_err("%s:failed NULL parameter\n", __func__);
		return -EINVAL;
	}
	vdev = video_devdata(file);
	sd = vdev_to_v4l2_subdev(vdev);
	u32 = (struct msm_ois_cfg_data32 *)arg;
	parg = arg;

	switch (cmd) {
	case VIDIOC_MSM_OIS_CFG32:
		cmd = VIDIOC_MSM_OIS_CFG;
		ois_data.cfgtype = u32->cfgtype;

		switch (u32->cfgtype) {
		case CFG_OIS_CONTROL:
			ois_data.cfg.set_info.ois_params.setting_size =
				u32->cfg.set_info.ois_params.setting_size;
			ois_data.cfg.set_info.ois_params.i2c_addr =
				u32->cfg.set_info.ois_params.i2c_addr;
			ois_data.cfg.set_info.ois_params.i2c_freq_mode =
				u32->cfg.set_info.ois_params.i2c_freq_mode;
			ois_data.cfg.set_info.ois_params.i2c_addr_type =
				u32->cfg.set_info.ois_params.i2c_addr_type;
			ois_data.cfg.set_info.ois_params.i2c_data_type =
				u32->cfg.set_info.ois_params.i2c_data_type;
			ois_data.cfg.set_info.ois_params.settings =
				compat_ptr(u32->cfg.set_info.ois_params.
				settings);
			parg = &ois_data;
			break;
		case CFG_OIS_I2C_WRITE_SEQ_TABLE:
			if (copy_from_user(&settings32,
				(void *)compat_ptr(u32->cfg.settings),
				sizeof(
				struct msm_camera_i2c_seq_reg_setting32))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			settings.addr_type = settings32.addr_type;
			settings.delay = settings32.delay;
			settings.size = settings32.size;
			settings.reg_setting =
				compat_ptr(settings32.reg_setting);

			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;
		/*ASUS_BSP +++ bill_chen "Implement ois command for dit 3A"*/
		case CFG_OIS_I2C_WRITE_MODE:
			if (copy_from_user(&settings32,
				(void *)compat_ptr(u32->cfg.settings),
				sizeof(
				struct msm_camera_i2c_seq_reg_setting32))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			settings.addr_type = settings32.addr_type;
			settings.delay = settings32.delay;
			settings.size = settings32.size;
			settings.reg_setting =
				compat_ptr(settings32.reg_setting);

			ois_data.cfgtype = u32->cfgtype;
			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;
		/*ASUS_BSP --- bill_chen "Implement ois command for dit 3A"*/
		default:
			parg = &ois_data;
			break;
		}
		break;
	case VIDIOC_MSM_OIS_CFG:
		pr_err("%s: invalid cmd 0x%x received\n", __func__, cmd);
		return -EINVAL;
	}
	rc = msm_ois_subdev_ioctl(sd, cmd, parg);

	return rc;
}

static long msm_ois_subdev_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_ois_subdev_do_ioctl);
}
#endif

static int32_t msm_ois_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_ois_ctrl_t *msm_ois_t = NULL;
	/*ASUS_BSP +++  bill_chen "Implement ois"*/
	//struct msm_ois_vreg *vreg_cfg; 
	uint32_t id_info[3];
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_ois_board_info *ob_info = NULL;
	pr_err("%s: E", __func__);  
	/*ASUS_BSP --- bill_chen "Implement ois"*/
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}


	msm_ois_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!msm_ois_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	msm_ois_t_pointer = msm_ois_t; /*ASUS_BSP bill_chen "Implement ois"*/
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) { 
		pr_err("failed rc %d\n", rc);
		goto release_memory;
	} 

//	msm_ois_t->oboard_info = kzalloc(sizeof(
//		struct msm_ois_board_info), GFP_KERNEL);
//	if (!msm_ois_t->oboard_info) {
//		kfree(msm_ois_t);
//		return -ENOMEM;
//	}


	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_ois_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_ois_t->cci_master, rc);
	if (rc < 0 || msm_ois_t->cci_master >= MASTER_MAX) {
		pr_err("failed rc %d\n", rc);
		goto release_memory;
	}

	/*ASUS_BSP +++ bill_chen "Implement ois"*/
#if 0
	if (of_find_property((&pdev->dev)->of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &msm_ois_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data((&pdev->dev)->of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			pr_err("failed rc %d\n", rc);
			goto release_memory;
		}
	}

	rc = msm_sensor_driver_get_gpio_data(&(msm_ois_t->gconf),
		(&pdev->dev)->of_node);
	if (-ENODEV == rc) {
		pr_notice("No valid OIS GPIOs data\n");
	} else if (rc < 0) {
		pr_err("Error OIS GPIO\n");
	} else {
		msm_ois_t->cam_pinctrl_status = 1;
		rc = msm_camera_pinctrl_init(
			&(msm_ois_t->pinctrl_info), &(pdev->dev));
		if (rc < 0) {
			pr_err("ERR: Error in reading OIS pinctrl\n");
			msm_ois_t->cam_pinctrl_status = 0;
		}
	}

#endif
	/*ASUS_BSP --- bill_chen "Implement ois"*/
	msm_ois_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
	msm_ois_t->ois_mutex = &msm_ois_mutex;

	/* Set platform device handle */
	msm_ois_t->pdev = pdev;
	/* Set device type as platform device */
	msm_ois_t->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_ois_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_ois_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_ois_t->i2c_client.cci_client) {
		kfree(msm_ois_t->vreg_cfg.cam_vreg);
		rc = -ENOMEM;
		goto release_memory;
	}

	cci_client = msm_ois_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = msm_ois_t->cci_master;
	v4l2_subdev_init(&msm_ois_t->msm_sd.sd,
		msm_ois_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_ois_t->msm_sd.sd, msm_ois_t);
	msm_ois_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	msm_ois_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_ois_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_ois_t->msm_sd.sd.name), "msm_ois");
	media_entity_init(&msm_ois_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_ois_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_ois_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	msm_ois_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_ois_t->msm_sd);
	msm_ois_t->ois_state = OIS_DISABLE_STATE;
	msm_cam_copy_v4l2_subdev_fops(&msm_ois_v4l2_subdev_fops);
#ifdef CONFIG_COMPAT
	msm_ois_v4l2_subdev_fops.compat_ioctl32 =
		msm_ois_subdev_fops_ioctl;
#endif
	msm_ois_t->msm_sd.sd.devnode->fops =
		&msm_ois_v4l2_subdev_fops;

	/*ASUS_BSP +++ bill_chen "Implement ois"*/
	create_ois_status_proc_file();
	create_ois_cali_proc_file();
	create_ois_mode_proc_file();
	create_ois_rw_proc_file();
	create_ois_power_proc_file();
	create_ois_dac_proc_file();
	create_ois_debug_proc_file();
	create_ois_device_proc_file();
	create_ois_vm_proc_file();
	msm_ois_t->oboard_info = kzalloc(sizeof(
		struct msm_ois_board_info), GFP_KERNEL);
	if (!msm_ois_t->oboard_info) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
	}
	ob_info = msm_ois_t->oboard_info;
	power_info = &ob_info->power_info;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	msm_ois_t->userspace_probe = 0;

	power_info->dev = &pdev->dev;
	msm_ois_t->subdev_id = pdev->id;

	rc = msm_camera_get_clk_info(msm_ois_t->pdev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_clk_info rc %d", rc);
		//goto board_free;
	}

	rc = msm_ois_get_dt_data(msm_ois_t);
	if (rc < 0) {
		pr_err("%s msm_ois_get_dt_data fail rc = %d\n", __func__, rc);
	}

	if (msm_ois_t->userspace_probe == 0) {


		rc = of_property_read_u32_array((&pdev->dev)->of_node, "qcom,slave-id",
			id_info, 3);
		if (rc < 0) {
			pr_err("%s request slave id failed %d rc = %d\n", __func__, __LINE__, rc);
			//goto board_free;
		}

		rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,i2c-freq-mode",
			&cci_client->i2c_freq_mode);
		pr_err("%s: qcom,i2c_freq_mode %d, rc %d\n", __func__, cci_client->i2c_freq_mode, rc);
		if (rc < 0) {
			pr_err("%s qcom,i2c-freq-mode read fail. Setting to 0 %d\n",
				__func__, rc);
			cci_client->i2c_freq_mode = 0;
		}

		msm_ois_t->i2c_client.cci_client->sid = id_info[0] >> 1;
		msm_ois_t->sensor_id_reg_addr = id_info[1];
		msm_ois_t->sensor_id = id_info[2];
		pr_err("%s slave id = 0x%x\n", __func__, msm_ois_t->i2c_client.cci_client->sid);
	 	pr_err("%s sensor id reg addr = 0x%x\n", __func__, msm_ois_t->sensor_id_reg_addr);
	 	pr_err("%s sensor id addr = 0x%x\n", __func__, msm_ois_t->sensor_id);


		rc = msm_camera_power_up(power_info, msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
		}
		usleep_range(1000, 1000);
		rc = msm_ois_check_id(msm_ois_t);
		if (rc < 0) {
			pr_err("%s msm_ois_check_id fail %d rc = %d\n", __func__, __LINE__, rc);
			//goto power_down;
		} else {
			g_ois_status = 1;
		}
		pr_err("%s g_ois_status = %d\n", __func__, g_ois_status);

		rc = msm_camera_power_down(power_info, msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
		}





	}

    pr_err("%s: rc = %d X", __func__, rc);
	CDBG("Exit\n");
	return rc;
release_memory:
	kfree(msm_ois_t->oboard_info);
	kfree(msm_ois_t);
	return rc;
}

static const struct of_device_id msm_ois_i2c_dt_match[] = {
	{.compatible = "qcom,ois"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_i2c_dt_match);

static struct i2c_driver msm_ois_i2c_driver = {
	.id_table = msm_ois_i2c_id,
	.probe  = msm_ois_i2c_probe,
	.remove = __exit_p(msm_ois_i2c_remove),
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_i2c_dt_match,
	},
};

static const struct of_device_id msm_ois_dt_match[] = {
	{.compatible = "qcom,ois2", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_dt_match);

static struct platform_driver msm_ois_platform_driver = {
	.probe = msm_ois_platform_probe,
	.driver = {
		.name = "qcom,ois2",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_dt_match,
	},
};

static int __init msm_ois_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_register(&msm_ois_platform_driver);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_ois_i2c_driver);
}

static void __exit msm_ois_exit_module(void)
{
	platform_driver_unregister(&msm_ois_platform_driver);
	i2c_del_driver(&msm_ois_i2c_driver);
	return;
}

/*ASUS_BSP +++ bill_chen "Implement ois"*/
static int32_t msm_ois_check_vm(struct msm_ois_ctrl_t *o_ctrl) {
	int32_t rc = 0;
	uint16_t write_num = 0;
    pr_err("%s: E", __func__);
    o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(o_ctrl->i2c_client), 0x847F, 0x0C0C, MSM_CAMERA_I2C_WORD_DATA);
    if(rc < 0) {
        pr_err("%s: ois write i2c 0x847F fail", __func__);
		goto OIS_CHECK_FAIL;
    }

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(o_ctrl->i2c_client), 0x847F, 0x0C2C, MSM_CAMERA_I2C_WORD_DATA);
    if(rc < 0) {
        pr_err("%s: ois write i2c 0x847F fail", __func__);
		goto OIS_CHECK_FAIL;
    }

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(o_ctrl->i2c_client), 0x8417, 0x000A, MSM_CAMERA_I2C_WORD_DATA);
    if(rc < 0) {
        pr_err("%s: ois write i2c 0x8417 fail", __func__);
		goto OIS_CHECK_FAIL;
    }

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(o_ctrl->i2c_client), 0x8497, 0x000A, MSM_CAMERA_I2C_WORD_DATA);
    if(rc < 0) {
        pr_err("%s: ois write i2c 0x8497 fail", __func__);
		goto OIS_CHECK_FAIL;
    }

	usleep_range(100000, 110000);

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(o_ctrl->i2c_client), 0x843F, &write_num, MSM_CAMERA_I2C_WORD_DATA);

    pr_err("%s: ois read num: 0x843F = 0x%x (int16_t) = %d\n", __func__, write_num, (int16_t)write_num);
    if(rc < 0) {
        pr_err("%s: ois read i2c 0x843F fail", __func__);
		goto OIS_CHECK_FAIL;
    }

	if((int16_t)write_num < 1280) {
		rc = -1;
		pr_err("%s: ois read i2c 0x843F range check fail", __func__);
		goto OIS_CHECK_FAIL;
	}

	usleep_range(100000, 110000);

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(o_ctrl->i2c_client), 0x84BF, &write_num, MSM_CAMERA_I2C_WORD_DATA);

    pr_err("%s: ois read num: 0x84BF = 0x%x (int16_t) = %d\n", __func__, write_num, (int16_t)write_num);
    if(rc < 0) {
        pr_err("%s: ois read i2c 0x84BF fail", __func__);
		goto OIS_CHECK_FAIL;
    }

	if((int16_t)write_num < 1280) {
		rc = -1;
		pr_err("%s: ois read i2c 0x84BF range check fail", __func__);
		goto OIS_CHECK_FAIL;
	}


OIS_CHECK_FAIL:
    pr_err("%s: rc = %d X\n", __func__, rc);
    return rc;
}
static int32_t msm_ois_check_id(struct msm_ois_ctrl_t *o_ctrl) {
	int32_t rc = 0;
	uint16_t chipid = 0;
	//uint16_t expectid = 0x735;
	//uint16_t chipidAddress = 0x8200;
	uint16_t write_num = 0;
	uint16_t test_write_num = 0x1357;
    //pr_err("%s: E", __func__);
    o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
		&(o_ctrl->i2c_client), o_ctrl->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);

    pr_err("%s: ois read id: 0x%x expected id: 0x%x\n", __func__, chipid, o_ctrl->sensor_id);

    if(rc < 0) {
        pr_err("%s: ois read i2c id probe fail", __func__);
		goto OIS_CHECK_FAIL;
    }

    if(chipid != o_ctrl->sensor_id) {
        pr_err("%s: ois salave address not match probe fail", __func__);
		rc = -EFAULT;
        goto OIS_CHECK_FAIL;
    }

    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(o_ctrl->i2c_client), 0x8220, test_write_num, MSM_CAMERA_I2C_WORD_DATA);
    rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(o_ctrl->i2c_client), 0x8220, &write_num, MSM_CAMERA_I2C_WORD_DATA);

    pr_err("%s: ois read num: 0x%x expected num: 0x%x\n", __func__, write_num, test_write_num);

OIS_CHECK_FAIL:
    //pr_err("%s: rc = %d X\n", __func__, rc);
    return rc;
}

static int msm_ois_get_dt_data(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0, i = 0;
	struct msm_ois_board_info *ob_info;
	struct msm_camera_power_ctrl_t *power_info =
		&o_ctrl->oboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	int8_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;

	ob_info = o_ctrl->oboard_info;

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = o_ctrl->pdev->dev.of_node;

	if (!of_node) {
		pr_err("%s: %d of_node is NULL\n", __func__ , __LINE__);
		return -ENOMEM;
	}

	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					     &power_info->num_vreg);
	pr_err("%s power_info->num_vreg = %d\n", __func__, power_info->num_vreg);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	if (o_ctrl->userspace_probe == 0) {
		rc = msm_camera_get_dt_power_setting_data(of_node,
			power_info->cam_vreg, power_info->num_vreg,
			power_info);
		if (rc < 0)
			goto ERROR1;
	}

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		rc = -ENOMEM;
		goto ERROR2;
	}

	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	pr_err("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size > 0) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR3;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			pr_err("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
	return rc;
}

/** @brief read many word(two bytes) from file
*
*	@param filename the file to write
*	@param value the word which will store the calibration data from read file
*	@param size the size of write data
*
*/
static int Sysfs_read_word_seq(char *filename, int *value, int size)
{
	int i = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[size][5];
	ssize_t buf_size = 0;
	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			buf_size = fp->f_op->read(fp, buf[i], 5, &pos_lsts);
			if(buf_size < 5) {
				/* Set addr_limit of the current process back to its own */
				set_fs(old_fs);
				/* close file */
				filp_close(fp, NULL);
				return -1;
			}
			buf[i][4]='\0';
			sscanf(buf[i], "%x", &value[i]);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return -ENXIO;	/*No such device or address*/
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	return 0;
}

/** @brief read many byte from file
*
*	@param filename the file to write
*	@param value the byte which will store the calibration data from read file
*	@param size the size of write data
*
*/
static int Sysfs_read_byte_seq(char *filename, int *value, int size)
{
	int i = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[3];
	ssize_t read_size = 0;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			read_size = fp->f_op->read(fp, buf, 3, &pos_lsts);
			buf[2]='\0';
			if(read_size == 0) {
				break;
			}
			sscanf(buf, "%x", &value[i]);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return -ENXIO;	/*No such device or address*/
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	return i;
}


/** @brief write many words(two bytes)  to file
*
*	@param filename the file to write
*	@param value the word which will be written to file
*	@param size the size of write data
*
*/
static bool Sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[size][5];

	for(i=0; i<size; i++){
		sprintf(buf[i], "%04x", value[i]);
		buf[i][4] = ' ';
	}

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: openfail line = %d\n", __func__, __LINE__);
		return false;
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			fp->f_op->write(fp, buf[i], 5, &fp->f_pos);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* Close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);
	return true;
}

static int Sysfs_read_char_seq(char *filename, int *value, int size)
{
	int i = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[2];
	ssize_t read_size = 0;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			read_size = fp->f_op->read(fp, buf, 1, &pos_lsts);
			pr_err("ois_cali read_size = %ld, buf[0] = %c buf[1] = %c",
				read_size, buf[0], buf[1]);
			buf[1]='\0';
			if(read_size == 0) {
				break;
			}
			sscanf(buf, "%d", &value[i]);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* Close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return -ENXIO;	/*No such device or address*/
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	return i;
}

static bool Sysfs_write_debug_seq(char *filename, uint16_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[5];

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: openfail line = %d\n", __func__, __LINE__);
		return false;
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			sprintf(buf, "%04x", value[i]);
			if((i + 1) % 4 == 0)
				buf[4] = '\n';
			else
				buf[4] = ',';
			fp->f_op->write(fp, buf, 5, &fp->f_pos);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* Close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);
	return true;
}

/*ASUS_BSP --- bill_chen "Implement ois"*/

/*ASUS_BSP +++ bill_chen "Implement camera ois"*/
#define	STATUS_OIS_DAC_PROC_FILE	"driver/vcm"
static struct proc_dir_entry *status_proc_file;
static int ois_dac_proc_read(struct seq_file *buf, void *v)
{
	int dac = 0;
	pr_err("%s: ois proc read dac = %d +++\n", __func__, g_ois_dac);
	dac = g_ois_dac;

	seq_printf(buf, "%d\n", dac);
	pr_err("%s: ois proc read ---\n", __func__);
	return 0;
}

static int ois_dac_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_dac_proc_read, NULL);
}

static const struct file_operations ois_dac_fops = {
	.owner = THIS_MODULE,
	.open = ois_dac_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_ois_dac_proc_file(void)
{
	if(!g_ois_dac_created) {
		status_proc_file = proc_create(STATUS_OIS_DAC_PROC_FILE, 0666, NULL, &ois_dac_fops);

		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_dac_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_dac_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
}

#define	STATUS_OIS_STATUS_PROC_FILE	"driver/ois_status"
static int ois_status_proc_read(struct seq_file *buf, void *v)
{
	unsigned char status = 0;
	pr_err("%s: ois proc read g_ois_status = %d\n", __func__, g_ois_status);
	status = g_ois_status;

	seq_printf(buf, "%d\n", status);
	//pr_err("%s: ois proc read ---\n", __func__);
	return 0;
}

static int ois_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_status_proc_read, NULL);
}

static const struct file_operations ois_status_fops = {
	.owner = THIS_MODULE,
	.open = ois_status_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_ois_status_proc_file(void)
{
	if(!g_ois_status_created) {
		status_proc_file = proc_create(STATUS_OIS_STATUS_PROC_FILE, 0666, NULL, &ois_status_fops);

		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_status_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_status_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
}

#define	STATUS_OIS_VM_PROC_FILE	"driver/ois_vm"
static int ois_vm_proc_read(struct seq_file *buf, void *v)
{
	unsigned char status = 0;
	int32_t rc = 0;
	int delay_count = 200;
	int camera_open = 0;
	struct msm_ois_ctrl_t *msm_ois_t = msm_ois_t_pointer;
	pr_err("%s: +++\n", __func__);
	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	Sysfs_read_char_seq("/data/.tmp/ATD_START", &camera_open, 1);
	pr_err("%s: camera_open = %d\n", __func__, camera_open);

	while(g_ois_mode == 255 && camera_open == 1) {
		usleep_range(50000, 51000);
		if((--delay_count) == 0)
			break;
	}

	if(g_ois_status == 0) {
		status = g_ois_status;
		seq_printf(buf, "%d\n", status);
		pr_err("%s: g_ois_status = 0 ---\n", __func__);
		return 0;
	}

	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	mutex_lock(msm_ois_t->ois_mutex);
	if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
		pr_err("%s msm_camera_power_up +++ %d\n", __func__, __LINE__);
		g_ois_mode = 255;
		rc = msm_camera_power_up(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
		}
		pr_err("%s msm_camera_power_up --- %d\n", __func__, __LINE__);

		if(camera_open == 0) {
			gpio_set_value_cansleep(
			msm_ois_t->oboard_info->power_info.gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_HIGH);
		}

		usleep_range(1000, 2000);
		rc = msm_ois_check_id(msm_ois_t);
		if (rc < 0) {
			pr_err("%s msm_ois_check_id %d rc = %d\n", __func__, __LINE__, rc);
		}

		pr_err("%s: ois_init_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_init_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_init_setting_array),
			ois_init_setting_array);
		if (rc < 0) {
			pr_err("%s write init setting fail %d rc = %d\n", __func__, __LINE__, rc);
		}

		pr_err("%s: ois_write_eeprom_data_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_write_eeprom_data_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_write_eeprom_data_setting_array),
			ois_write_eeprom_data_setting_array);
		if (rc < 0) {
			pr_err("%s write eeprom data setting fail %d rc = %d\n", __func__, __LINE__, rc);
		}

		pr_err("%s: ois_write_calibration_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_write_calibration_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_write_calibration_setting_array),
			ois_write_calibration_setting_array);
		if (rc < 0) {
			pr_err("%s write calibration data setting fail %d rc = %d\n", __func__, __LINE__, rc);
		}

		pr_err("%s: ois_dsp_start_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_dsp_start_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_dsp_start_setting_array),
			ois_dsp_start_setting_array);
		if (rc < 0) {
			pr_err("%s write dsp start setting fail %d rc = %d\n", __func__, __LINE__, rc);
		}

		pr_err("%s: ois_disable_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_disable_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_disable_setting_array),
			ois_disable_setting_array);
		if (rc < 0) {
			pr_err("%s write disable setting fail %d rc = %d\n", __func__, __LINE__, rc);
		} else {
			g_ois_mode = 0;
		}
	}

	rc = msm_ois_check_vm(msm_ois_t);
	if (rc < 0) {
		pr_err("%s msm_ois_check_vm fail %d rc = %d\n", __func__, __LINE__, rc);
		status = 0;
	} else {
		status = 1;
	}

	if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
		pr_err("%s msm_camera_power_down +++ %d\n", __func__, __LINE__);
		rc = msm_camera_power_down(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
		}
		if(camera_open == 0) {
			gpio_set_value_cansleep(
			msm_ois_t->oboard_info->power_info.gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VIO],
			GPIO_OUT_LOW);
		}
		g_ois_mode = 255;
		pr_err("%s msm_camera_power_down --- %d\n", __func__, __LINE__);
	}
	mutex_unlock(msm_ois_t->ois_mutex);

	seq_printf(buf, "%d\n", status);
	pr_err("%s: ois proc read ---\n", __func__);
	return 0;
}

static int ois_vm_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_vm_proc_read, NULL);
}

static const struct file_operations ois_vm_fops = {
	.owner = THIS_MODULE,
	.open = ois_vm_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_ois_vm_proc_file(void)
{
	if(!g_ois_vm_created) {
		status_proc_file = proc_create(STATUS_OIS_VM_PROC_FILE, 0666, NULL, &ois_vm_fops);

		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_vm_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_vm_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
}

#define	STATUS_OIS_CALI_PROC_FILE	"driver/ois_cali"
static int ois_cali_proc_read(struct seq_file *buf, void *v)
{
	int32_t rc = 0;
	int32_t cali_rc = 1;
	int16_t gyro_criteria = 2162;
	int delay_count = 200;
	int camera_open = 0;
	struct msm_ois_ctrl_t *msm_ois_t = msm_ois_t_pointer;
	pr_err("%s: +++\n", __func__);
	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	Sysfs_read_char_seq("/data/.tmp/ATD_START", &camera_open, 1);
	pr_err("%s: camera_open = %d\n", __func__, camera_open);

	while(g_ois_mode == 255 && camera_open == 1) {
		usleep_range(50000, 51000);
		if((--delay_count) == 0)
			break;
	}

#if 0
	if(delay_count == 0) {
		cali_rc = 0;
		seq_printf(buf, "%d\n", cali_rc);
		pr_err("%s: fail delay_count = 0\n", __func__);
		return 0;
	}
#endif
	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	mutex_lock(msm_ois_t->ois_mutex);
	if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
		rc = msm_camera_power_up(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
			cali_rc = 0;
		}
		usleep_range(1000, 2000);
		rc = msm_ois_check_id(msm_ois_t);
		if (rc < 0) {
			pr_err("%s msm_ois_check_id %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}

		pr_err("%s: ois_init_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_init_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_init_setting_array),
			ois_init_setting_array);
		if (rc < 0) {
			pr_err("%s write init setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}

		pr_err("%s: ois_dsp_start_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_dsp_start_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_dsp_start_setting_array),
			ois_dsp_start_setting_array);
		if (rc < 0) {
			pr_err("%s write dsp start setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}

	pr_err("%s: ois_read_calibration_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_read_calibration_setting_array));
	rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_read_calibration_setting_array),
		ois_read_calibration_setting_array);
	if (rc < 0) {
		pr_err("%s ois_read_calibration_setting_array %d rc = %d\n", __func__, __LINE__, rc);
		cali_rc = 0;
	}

	pr_err("%s: OIS_CALIBRATION_GRYO_OFFSET_X[0] = 0x%x\n", __func__, OIS_CALIBRATION_GRYO_OFFSET_X[0]);
	pr_err("%s: OIS_CALIBRATION_GRYO_OFFSET_Y[0] = 0x%x\n", __func__, OIS_CALIBRATION_GRYO_OFFSET_Y[0]);

	if((int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[0] > gyro_criteria ||
		(int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[0] < -gyro_criteria) {
		pr_err("%s: ois calibration criteria gyro offset X fail\n", __func__);
		cali_rc = 0;
	}

	if((int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[0] > gyro_criteria ||
		(int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[0] < -gyro_criteria) {
		pr_err("%s: ois calibration criteria gyro offset Y fail\n", __func__);
		cali_rc = 0;
	}

	if((int16_t)OIS_CALIBRATION_GRYO_OFFSET_X[0] == 0 &&
		(int16_t)OIS_CALIBRATION_GRYO_OFFSET_Y[0] == 0) {
		pr_err("%s: ois calibration criteria gyro offset X = 0 Y = 0\n", __func__);
	}

	if(cali_rc != 0) {
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_write_calibration_setting_array),
			ois_write_calibration_setting_array);
		pr_err("%s: write gyro calibration data to register\n", __func__);
		if (rc < 0) {
			pr_err("%s write gyro calibration data to register fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}

	if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
		rc = msm_camera_power_down(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
			&msm_ois_t->i2c_client);
		if (rc) {
			pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
			cali_rc = 0;
		}
	}
	mutex_unlock(msm_ois_t->ois_mutex);
	seq_printf(buf, "%d\n", cali_rc);
	pr_err("%s: ---\n", __func__);
	return 0;
}

static int ois_cali_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_cali_proc_read, NULL);
}

static const struct file_operations ois_cali_fops = {
	.owner = THIS_MODULE,
	.open = ois_cali_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_ois_cali_proc_file(void)
{
	if(!g_ois_cali_created) {
		status_proc_file = proc_create(STATUS_OIS_CALI_PROC_FILE, 0666, NULL, &ois_cali_fops);

		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_cali_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_cali_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
}

#define	STATUS_OIS_MODE_PROC_FILE	"driver/ois_mode"
static int ois_mode_proc_read(struct seq_file *buf, void *v)
{
	unsigned char mode = 0;
	pr_err("%s: ois proc read g_ois_mode = %d +++\n", __func__, g_ois_mode);
	mode = g_ois_mode;

	seq_printf(buf, "%d\n", mode);
	pr_err("%s: ois proc read ---\n", __func__);
	return 0;
}

static int ois_mode_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_mode_proc_read, NULL);
}

static const struct file_operations ois_mode_fops = {
	.owner = THIS_MODULE,
	.open = ois_mode_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_ois_mode_proc_file(void)
{
	if(!g_ois_mode_created) {
		status_proc_file = proc_create(STATUS_OIS_MODE_PROC_FILE, 0666, NULL, &ois_mode_fops);

		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_mode_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_mode_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
}

static int ois_rw_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "0x%x=0x%x\n",g_reg,g_value);
	return 0;
}

static int ois_rw_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_rw_read, NULL);
}

static ssize_t ois_rw_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1,value = -1;
	int rc, len;
	uint16_t read_num = 0;
	uint16_t ori_sid = 0;
	struct msm_ois_ctrl_t *msm_ois_t = msm_ois_t_pointer;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	pr_err("%s: +++\n", __func__);

	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
			return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x %x", &reg, &value);
	*ppos=len;
	pr_err("ois write reg=0x%x value=0x%x\n", reg, value);
	g_reg = reg;
	g_value = value;
	mutex_lock(msm_ois_t->ois_mutex);
	if (reg == 0xA0) {
		ori_sid = msm_ois_t->i2c_client.cci_client->sid;
		pr_err("A0 ois ori_sid = 0x%x\n", ori_sid);
		msm_ois_t->i2c_client.cci_client->sid = 0x50;
		//rc = msm_ois_t->i2c_client.i2c_func_tbl->i2c_write(
			//&(msm_ois_t->i2c_client), value,
			//0xffff, MSM_CAMERA_I2C_WORD_DATA);
		usleep_range(10000, 11000);
		rc = msm_ois_t->i2c_client.i2c_func_tbl->i2c_read(
			&(msm_ois_t->i2c_client), value,
			&read_num, MSM_CAMERA_I2C_WORD_DATA);
		g_value = read_num;
		pr_err("A0 ois read reg=0x%x read_num=0x%x\n", value, read_num);
		msm_ois_t->i2c_client.cci_client->sid = ori_sid;
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			mutex_unlock(msm_ois_t->ois_mutex);
			return rc;
		}
	} else if (reg == 0xF090) {
		reg_setting.reg_addr = reg;
		reg_setting.reg_data[0] = 0;
		reg_setting.reg_data[1] = value / 256;
		reg_setting.reg_data[2] = value % 256;
		reg_setting.reg_data_size = 3;
		pr_err("%s: vcm reg_setting.reg_addr = 0x%x, reg_data[0] = 0x%x, reg_setting.reg_data[1] = 0x%x, reg_setting.reg_data[2] = 0x%x\n", __func__, reg_setting.reg_addr, reg_setting.reg_data[0], reg_setting.reg_data[1], reg_setting.reg_data[2]);
		rc = msm_ois_t->i2c_client.i2c_func_tbl->
			i2c_write_seq(&msm_ois_t->i2c_client,
			reg_setting.reg_addr,
			reg_setting.reg_data,
			reg_setting.reg_data_size);	
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x rc = %d\n",
				 __func__, reg, rc);
			mutex_unlock(msm_ois_t->ois_mutex);
			return rc;
		}
	} else if (reg != -1 && value != -1) {
		pr_err("ois write reg=0x%x value=0x%x reverse_value = 0x%x\n ", reg, value,
			((value & 0xff00) >> 8) + ((value & 0x00ff) << 8));
		rc = msm_ois_t->i2c_client.i2c_func_tbl->i2c_write(
			&(msm_ois_t->i2c_client), reg,
			((value & 0xff00) >> 8) + ((value & 0x00ff) << 8), MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__, reg, value);
			mutex_unlock(msm_ois_t->ois_mutex);
			return rc;
		}
	} else if (reg != -1) {
		rc = msm_ois_t->i2c_client.i2c_func_tbl->i2c_read(
			&(msm_ois_t->i2c_client), reg,
			&read_num, MSM_CAMERA_I2C_WORD_DATA);
		g_value = read_num;
		pr_err("ois read reg=0x%x read_num=0x%x\n", reg, read_num);
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			mutex_unlock(msm_ois_t->ois_mutex);
			return rc;
		}
	}
	mutex_unlock(msm_ois_t->ois_mutex);
	pr_err("%s: ---\n", __func__);
	return len;
}


static const struct file_operations ois_rw_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_rw_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_rw_write,
};

#define	OIS_RW_PROC_FILE	"driver/ois_rw"
static void create_ois_rw_proc_file(void){
	
	if(!g_ois_rw_created) {
	    status_proc_file = proc_create(OIS_RW_PROC_FILE, 0666, NULL, &ois_rw_proc_fops);
	    if (status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_rw_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_rw_created = 0;
	    }
	} else {  
        pr_info("File Exist!\n");  
    }
}


static int ois_power_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_power_state);
	return 0;
}

static int ois_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_power_read, NULL);
}

static ssize_t ois_power_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int32_t cali_rc = 1;
	int value = -1;
	int rc, len;
	int delay_count = 200;
	int camera_open = 0;
	struct msm_ois_ctrl_t *msm_ois_t = msm_ois_t_pointer;
	pr_err("%s: +++\n", __func__);
	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);

	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
			return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%d", &value);
	*ppos=len;
	pr_err("%s: ois write value=0x%x\n", __func__, value);


	Sysfs_read_char_seq("/data/.tmp/ATD_START", &camera_open, 1);
	pr_err("%s: camera_open = %d\n", __func__, camera_open);

	if(value != 4 && value != 5 && value != 6) {
		while(g_ois_mode == 255 && camera_open == 1) {
			usleep_range(50000, 51000);
			if((--delay_count) == 0)
				break;
		}
	}
	pr_err("%s: LINE = %d\n", __func__, __LINE__);
	mutex_lock(msm_ois_t->ois_mutex);
	if(value == 5) {
		pr_err("%s: LINE = %d\n", __func__, __LINE__);
		if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
			pr_err("%s: LINE = %d\n", __func__, __LINE__);
			rc = msm_camera_power_up(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
				&msm_ois_t->i2c_client);
			if (rc) {
				pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
				cali_rc = 0;
			}
		}
	}

	if(value == 4) {
		pr_err("%s: LINE = %d\n", __func__, __LINE__);
		if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
			pr_err("%s: LINE = %d\n", __func__, __LINE__);
			rc = msm_camera_power_down(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
				&msm_ois_t->i2c_client);
			if (rc) {
				pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
				cali_rc = 0;
			}
		}
	}

	if(value == 6) {
		usleep_range(1000, 2000);
		rc = msm_ois_check_id(msm_ois_t);
		if (rc < 0) {
			pr_err("%s msm_ois_check_id %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}

		pr_err("%s: ois_init_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_init_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_init_setting_array),
			ois_init_setting_array);
		if (rc < 0) {
			pr_err("%s write init setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}

		pr_err("%s: ois_dsp_start_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_dsp_start_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_dsp_start_setting_array),
			ois_dsp_start_setting_array);
		if (rc < 0) {
			pr_err("%s write dsp start setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}

	if(value == 0) {
		pr_err("%s: ois_disable_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_disable_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_disable_setting_array),
			ois_disable_setting_array);
		if (rc < 0) {
			pr_err("%s write ois disable setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}

	if(value == 1) {
		pr_err("%s: ois_enable_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_enable_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_enable_setting_array),
			ois_enable_setting_array);
		if (rc < 0) {
			pr_err("%s write ois enable setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
		pr_err("%s: ois_movie_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_movie_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_movie_setting_array),
			ois_movie_setting_array);
		if (rc < 0) {
			pr_err("%s write ois movie setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}

	if(value == 2) {
		pr_err("%s: ois_enable_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_enable_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_enable_setting_array),
			ois_enable_setting_array);
		if (rc < 0) {
			pr_err("%s write ois enable setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
		pr_err("%s: ois_still_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_still_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_still_setting_array),
			ois_still_setting_array);
		if (rc < 0) {
			pr_err("%s write ois still setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}


	if(value == 3) {
		pr_err("%s: ois_enable_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_enable_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_enable_setting_array),
			ois_enable_setting_array);
		if (rc < 0) {
			pr_err("%s write ois enable setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
		pr_err("%s: ois_test_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_test_setting_array));
		rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_test_setting_array),
			ois_test_setting_array);
		if (rc < 0) {
			pr_err("%s write ois test setting fail %d rc = %d\n", __func__, __LINE__, rc);
			cali_rc = 0;
		}
	}
	mutex_unlock(msm_ois_t->ois_mutex);

	if(cali_rc == 1)
		g_power_state = value;
	pr_err("%s: ---\n", __func__);
	return len;
}


static const struct file_operations ois_power_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_power_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_power_write,
};

#define	OIS_POWER_PROC_FILE	"driver/ois_power"
static void create_ois_power_proc_file(void){

	if(!g_ois_power_created) {
	    status_proc_file = proc_create(OIS_POWER_PROC_FILE, 0666, NULL, &ois_power_proc_fops);
	    if (status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_power_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_power_created = 0;
	    }
	} else {
        pr_info("File Exist!\n");
    }
}

static int ois_debug_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_ois_debug_status);
	return 0;
}

static int ois_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_debug_read, NULL);
}

static ssize_t ois_debug_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1,value = -1;
	int rc, len, i, j;
	int delay_count = 200;
	int camera_open = 0;
	bool file_rc = false;
	struct msm_ois_ctrl_t *msm_ois_t = msm_ois_t_pointer;
	int gyro_acc_array_size = ARRAY_SIZE(OIS_GYRO_ACC_VALUE);
	g_ois_debug_status = 1;
	pr_err("%s: +++\n", __func__);
	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	Sysfs_read_char_seq("/data/.tmp/ATD_START", &camera_open, 1);
	pr_err("%s: camera_open = %d\n", __func__, camera_open);

	while(g_ois_mode == 255 && camera_open == 1) {
		usleep_range(50000, 51000);
		if((--delay_count) == 0)
			break;
	}

	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
			return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%d %d", &reg, &value);
	*ppos=len;
	pr_err("%s: ois write reg=0x%x value=0x%x\n", __func__, reg, value);
	g_reg = reg;
	g_value = value;

	pr_err("%s: ois_state = %d\n", __func__, msm_ois_t->ois_state);
	mutex_lock(msm_ois_t->ois_mutex);
	if (reg == 1) {
		if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
			rc = msm_camera_power_up(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
				&msm_ois_t->i2c_client);
			if (rc) {
				pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
				g_ois_debug_status = 0;
			}
			usleep_range(1000, 2000);
			rc = msm_ois_check_id(msm_ois_t);
			if (rc < 0) {
				pr_err("%s msm_ois_check_id %d rc = %d\n", __func__, __LINE__, rc);
				g_ois_debug_status = 0;
			}

			pr_err("%s: ois_init_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_init_setting_array));
			rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_init_setting_array),
				ois_init_setting_array);
			if (rc < 0) {
				pr_err("%s write init setting fail %d rc = %d\n", __func__, __LINE__, rc);
				g_ois_debug_status = 0;
			}

			pr_err("%s: ois_dsp_start_setting_array size = %ld\n", __func__, ARRAY_SIZE(ois_dsp_start_setting_array));
			rc = msm_ois_write_settings(msm_ois_t, ARRAY_SIZE(ois_dsp_start_setting_array),
				ois_dsp_start_setting_array);
			if (rc < 0) {
				pr_err("%s write dsp start setting fail %d rc = %d\n", __func__, __LINE__, rc);
				g_ois_debug_status = 0;
			}
		}

		usleep_range(10000, 11000);
		pr_err("%s: ois debug_calibration_data ARRAY_SIZE(OIS_GYRO_ACC_VALUE) = %d +++", __func__, gyro_acc_array_size);
		for(i = 0; i < value; i++) {
			for(j = 0; j < gyro_acc_array_size; j++) {
				rc = msm_ois_t->i2c_client.i2c_func_tbl->i2c_read(
					&(msm_ois_t->i2c_client), OIS_GYRO_ACC_VALUE[j],
					&(debug_calibration_data[(gyro_acc_array_size * i) + j]), MSM_CAMERA_I2C_WORD_DATA);
				if (rc < 0)
					break;
			}
			if (rc < 0) {
				pr_err("%s: read 0x%x fail rc = %d\n", __func__, OIS_GYRO_ACC_VALUE[j], rc);
				g_ois_debug_status = 0;
				break;
			}
		}
		pr_err("%s: ois debug_calibration_data ---", __func__);
		if(g_ois_debug_status == 1) {
			pr_err("%s: Sysfs_write_debug_seq /data/data/OIS_debug", __func__);
			file_rc = Sysfs_write_debug_seq("/data/data/OIS_debug", debug_calibration_data, value * gyro_acc_array_size);
			if(file_rc == 0) {
				g_ois_debug_status = 0;
				pr_err("%s: read file fail\n", __func__);
			}
		}

		if(msm_ois_t->ois_state != OIS_ENABLE_STATE && msm_ois_t->ois_state != OIS_OPS_ACTIVE) {
			rc = msm_camera_power_down(&(msm_ois_t->oboard_info->power_info), msm_ois_t->ois_device_type,
				&msm_ois_t->i2c_client);
			if (rc) {
				pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
				g_ois_debug_status = 0;
			}
		}
	}
	mutex_unlock(msm_ois_t->ois_mutex);
	pr_err("%s: ---\n", __func__);
	return len;
}


static const struct file_operations ois_debug_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_debug_write,
};

#define	OIS_DEBUG_PROC_FILE	"driver/ois_debug"
static void create_ois_debug_proc_file(void){

	if(!g_ois_debug_created) {
	    status_proc_file = proc_create(OIS_DEBUG_PROC_FILE, 0666, NULL, &ois_debug_proc_fops);
	    if (status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_debug_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_debug_created = 0;
	    }
	} else {
        pr_info("File Exist!\n");
    }
}

static int ois_device_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n",ois_subdev_string);
	return 0;
}

static int ois_device_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_device_read, NULL);
}

static ssize_t ois_device_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int len;
	//pr_err("%s: +++\n", __func__);
	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
			return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%s", ois_subdev_string);
	*ppos=len;
	pr_err("ois write subdev=%s\n", ois_subdev_string);
	//pr_err("%s: ---\n", __func__);
	return len;
}


static const struct file_operations ois_device_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_device_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_device_write,
};

#define	OIS_DEVICE_PROC_FILE	"driver/ois_device"
static void create_ois_device_proc_file(void){

	if(!g_ois_device_created) {
	    status_proc_file = proc_create(OIS_DEVICE_PROC_FILE, 0666, NULL, &ois_device_proc_fops);
	    if (status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_ois_device_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_ois_device_created = 0;
	    }
	} else {
        pr_info("File Exist!\n");
    }
}

/*ASUS_BSP --- bill_chen "Implement camera ois"*/

module_init(msm_ois_init_module);
module_exit(msm_ois_exit_module);
MODULE_DESCRIPTION("MSM OIS");
MODULE_LICENSE("GPL v2");
