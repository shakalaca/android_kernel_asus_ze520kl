/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "laser_focus_i2c.h"
#include "laser_log.h"

/** @brief Write one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/
#define CCI_DELAY	5


//CCI-I2C functions for HPTG+++

extern void swap_data(uint16_t* register_data);

int CCI_I2C_WrByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t i2c_write_data)
{
	int cnt=0;
	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;	
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}
	
	do{
		status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
					i2c_write_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (status < 0)
			msleep(CCI_DELAY);
			
	}while(status < 0&& ++cnt<1);

	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x, retry %d \n", __func__, register_addr, i2c_write_data, cnt);	
	return status;
}

/** @brief Write one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/
int CCI_I2C_WrWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t write_data)
{
	int cnt=0;
	int status;
	uint16_t i2c_write_data = write_data;
	struct msm_camera_i2c_client *sensor_i2c_client;	
	swap_data(&i2c_write_data);
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}
	
	do{
		status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
					i2c_write_data, MSM_CAMERA_I2C_WORD_DATA);
		if (status < 0) 
			msleep(CCI_DELAY);
		
	}while(status < 0&& ++cnt<1);

	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x, retry %d \n", __func__, register_addr, i2c_write_data, cnt);
	
	return status;
}

/** @brief Write seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the value which will be written
*	@param num_bytes the size of write data
*
*/
int CCI_I2C_WrByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_write_data, uint32_t num_byte)
{
	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write_seq(sensor_i2c_client, register_addr, 
		i2c_write_data, num_byte);

	if (status < 0) 
		LOG_Handler(LOG_ERR, "%s: write register(0x%x) failed\n", __func__, register_addr);

	return status;
}


/** @brief Read one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	int cnt=0;
       int status;      
       struct msm_camera_i2c_client *sensor_i2c_client;
		
       sensor_i2c_client = dev_t->i2c_client;
       if (!sensor_i2c_client) {
               LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
               return -EINVAL;
       }
	   
	do{
	       status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
	                       i2c_read_data, MSM_CAMERA_I2C_BYTE_DATA);	
		   if (status < 0)
	               msleep(CCI_DELAY);
	        
	}while(status < 0 && ++cnt<1);

       LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x, retry %d \n", __func__, register_addr, *i2c_read_data, cnt);
	
       return status;
}

/** @brief Read one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	int cnt=0;
	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}
	
	do{	
		status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
			i2c_read_data, MSM_CAMERA_I2C_WORD_DATA);	
		if (status < 0)
			msleep(CCI_DELAY);

	}while(status < 0 && ++cnt<1);

	swap_data(i2c_read_data);

	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x, retry %d \n", __func__, register_addr, *i2c_read_data, cnt);
	return status;
}

/** @brief Read two words via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_byte number of the bytes which will be read
*
*/
int CCI_I2C_RdDWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint32_t *i2c_read_data)
{
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = 4;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	*i2c_read_data=((uint32_t)reg_setting.reg_data[0]<<24)|((uint32_t)reg_setting.reg_data[1]<<16)|((uint32_t)reg_setting.reg_data[2]<<8)|((uint32_t)reg_setting.reg_data[3]);
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

/** @brief Read seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_bytes the size of read data
*
*/
int CCI_I2C_RdByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_read_data, uint32_t num_byte)
{
	int status;
	uint8_t buf[num_byte];
	struct msm_camera_i2c_client *sensor_i2c_client;
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		buf, num_byte);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, i2c_read_data);

	return status;
}

//CCI-I2C functions for HPTG---



//CCI-I2C functions for ST+++


extern struct msm_laser_focus_ctrl_t *laserSTx_t;

int ST_CCI_WrByte(uint32_t register_addr, uint16_t i2c_write_data)
{
	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;
	
	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

int ST_CCI_RdByte(uint32_t register_addr, uint16_t *i2c_read_data)
{
        int status;
        struct msm_camera_i2c_client *sensor_i2c_client;
		
        sensor_i2c_client = laserSTx_t->i2c_client;
        if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
        }

        status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
                        i2c_read_data, MSM_CAMERA_I2C_BYTE_DATA);
		
        if (status < 0) {
                LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
                return status;
        }
        LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);
        return status;
}

int ST_CCI_WrWord(uint32_t register_addr, uint16_t i2c_write_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

int ST_CCI_RdWord(uint32_t register_addr, uint16_t *i2c_read_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
		i2c_read_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

int ST_CCI_WrDWord(uint32_t register_addr, uint32_t i2c_write_data){
	int status;
	uint16_t high_val, low_val;
	
	high_val = (uint16_t)((i2c_write_data&0xFFFF0000)>>16);
	low_val = (uint16_t)(i2c_write_data&0x0000FFFF);
	
	status = ST_CCI_WrWord(register_addr, high_val);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	status = ST_CCI_WrWord(register_addr+2, low_val);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr+2);
		return status;
	}

	LOG_Handler(LOG_REG, "%s: wirte register(0x%x) : 0x%x\n", __func__, register_addr, i2c_write_data);	

	return status;
}

int ST_CCI_RdDWord(uint32_t register_addr, uint32_t *i2c_read_data, uint16_t num_byte)
{
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = num_byte;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	*i2c_read_data=((uint32_t)reg_setting.reg_data[0]<<24)|((uint32_t)reg_setting.reg_data[1]<<16)|((uint32_t)reg_setting.reg_data[2]<<8)|((uint32_t)reg_setting.reg_data[3]);
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

int ST_CCI_WrMulti(uint32_t register_addr, uint8_t *i2c_read_data, uint16_t num_byte){

	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}	
	status = sensor_i2c_client->i2c_func_tbl->i2c_write_seq(sensor_i2c_client, register_addr, i2c_read_data, num_byte);

	return status;	

}

int ST_CCI_RdMultiWord(uint32_t register_addr, uint8_t *i2c_read_data, uint16_t num_byte){

	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = laserSTx_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}	
	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, i2c_read_data, num_byte);

	return status;
}


int ST_CCI_UpdateByte(uint32_t register_addr, uint8_t AndData, uint8_t OrData){
	int status;
	uint16_t i2c_read_data, i2c_write_data;

	status = ST_CCI_RdByte(register_addr, &i2c_read_data);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s failed\n", __func__);
		return status;
	}

	i2c_write_data = ((uint8_t)(i2c_read_data&0x00FF)&AndData) | OrData;

	status = ST_CCI_WrByte(register_addr, i2c_write_data);

	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

 	LOG_Handler(LOG_REG, "%s: update register(0x%x) from 0x%x to 0x%x(AndData:0x%x;OrData:0x%x)\n", __func__, register_addr, i2c_read_data, i2c_write_data,AndData,OrData);

	return status;
}

//CCI-I2C functions for ST---

