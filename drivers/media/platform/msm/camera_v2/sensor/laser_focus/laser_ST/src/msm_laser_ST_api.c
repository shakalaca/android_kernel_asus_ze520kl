/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
//#define DEBUG
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_laser_focus.h"
#include "laser_sysfs.h"
#include "msm_cci.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "msm_laser_ST_api.h"
#include <linux/of_gpio.h>
#include "laser_log.h"
#include "vl53l0_types.h"
#include "vl53l0_api.h"


#define VL6180_API_ERROR_COUNT_MAX 5
#define LOG_DISPLAY_COUNT 50

//DEFINE_MSM_MUTEX(msm_laser_focus_mutex);

extern int g_ftm_mode;
extern int g_factory;


struct msm_laser_focus_ctrl_t *laserSTx_t = NULL;
VL53L0_Dev_t* ST_data_t = NULL;
struct timeval timer, timer2;
bool camera_on_flag = false;
int vl6180x_check_status = 0;
int log_count = 0;

uint8_t	g_preRange=18, g_finalRange=14;

uint32_t	 g_signal_Kcps=100, g_sigma_mm=60, g_time_budget_ms=33;
#define FIXPOINT_SHIFT	65536

struct mutex vl6180x_mutex;

uint8_t STerrorStatus = 16;
uint16_t DMax = 0;
int ErrCode = 0;
uint16_t Range =0;
bool ClosingLaser =false;
extern bool timedMeasure;
struct delayed_work		STkeepMeasure;
struct workqueue_struct*	STMeasure_wq;
struct delayed_work		STMeasure_wk;

static int ATD_status;


static int VL6180x_device_Load_Calibration_Value(void){
	int8_t status = 0;
	int offset = 0;
	uint32_t cross_talk = 0;
	//uint8_t VhvSettings, PhaseCal, count, isApertureSpads;
	
	//FixPoint1616_t CalibratedPosition, SignalRateRtnMegaCps;

	uint32_t refKdata[REF_K_DATA_NUMBER];
	uint32_t dmaxKdata[DMAX_K_DATA_NUMBER];

	uint32_t signal,sigma,budget;
	
	if (laserSTx_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION){
		/* Read Calibration data */
		
			status = Laser_sysfs_read_offset(&offset);
		if(status==0)
			status = Laser_sysfs_read_xtalk(&cross_talk);
		if(status==0)
			status = Sysfs_read_Dword_seq(LASER_K_REF_FILE,refKdata,REF_K_DATA_NUMBER);
		if(status==0)
			status = Sysfs_read_Dword_seq(LASER_K_DMAX_FILE,dmaxKdata,DMAX_K_DATA_NUMBER);

		if(status){
			pr_err("%s: fail at reading (%d)\n",__FUNCTION__ ,status);
			return status;
		}
		
			status =VL53L0_SetOffsetCalibrationDataMicroMeter(ST_data_t, offset);
		if(status==0)
			status = VL53L0_SetXTalkCompensationEnable(ST_data_t, 1);
		if(status==0)
			status = VL53L0_SetXTalkCompensationRateMegaCps(ST_data_t, cross_talk);
		if(status==0)
			status =	VL53L0_SetReferenceSpads(ST_data_t, refKdata[0], refKdata[1]);
		if(status==0)
			status =	VL53L0_SetRefCalibration(ST_data_t, refKdata[2], refKdata[3]);
		if(status==0)
			status =	VL53L0_SetDmaxCalParameters(ST_data_t, VL6180_CROSSTALK_CAL_RANGE, dmaxKdata[1], 9*65536);
		if(status){
			pr_err("%s fail at setting (%d)\n",__FUNCTION__ ,status);
			return status;
		}	
		printk("%s: offset(%d) Xtalk(%d) Spad(%d  %d) Tmp(%d %d) Dmax(%d %d)\n",__func__
			,offset, cross_talk
			,refKdata[0],refKdata[1]
			,refKdata[2],refKdata[3]
			,dmaxKdata[0],dmaxKdata[1]);

	    
		        status = VL53L0_SetLimitCheckEnable(ST_data_t,
		        		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
		    if (status == 0) {
		        status = VL53L0_SetLimitCheckEnable(ST_data_t,
		        		VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
		    }				
		    if (status == 0) {
		        status = VL53L0_SetLimitCheckValue(ST_data_t,
		        		VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(g_signal_Kcps*FIXPOINT_SHIFT)/1000);
				//(g_signal_Kcps*FIXPOINT_SHIFT)/1000
				//(FixPoint1616_t)(0.1*65536)
			}			
		    if (status == 0) {
		        status = VL53L0_SetLimitCheckValue(ST_data_t,
		        		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		g_sigma_mm*FIXPOINT_SHIFT);			
		    }
		    if (status == 0) {
		        status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(ST_data_t,
		        		(g_time_budget_ms)*1000);
			}
		printk("set raw signal(%d), sigma(%d), budget_ms(%d)\n",g_signal_Kcps,g_sigma_mm,g_time_budget_ms);
		
		VL53L0_GetLimitCheckValue(ST_data_t,VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,&signal);
		
		VL53L0_GetLimitCheckValue(ST_data_t,VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,&sigma);

		VL53L0_GetMeasurementTimingBudgetMicroSeconds(ST_data_t,&budget);

		printk("get signal(%d), sigma(%d), budget_us(%d)\n",signal,sigma,budget);

		
	}
	API_DBG("%s: success\n",__func__);
	return status;
}



int Laser_device_on(void){
	int rc=0;
	laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
	//rc = PowerUp(laserSTx_t);
	rc = dev_cci_init(laserSTx_t);
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return -EIO;
	} 
	return rc;

}
void Laser_device_off(void){
	ClosingLaser = true;
	dev_cci_deinit(laserSTx_t);
	//PowerDown(laserSTx_t);
	laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
}

int Laser_init_data(void){
	int rc=0;
	API_DBG("%s: VL53L0_DataInit Start\n", __func__);
	rc = VL53L0_DataInit(ST_data_t);
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return rc;
	}
	API_DBG("%s: VL53L0_DataInit Success\n", __func__);
	return rc;
}

int Laser_prepare(void){
	int rc=0;
	uint32_t signal,sigma,budget;
		
	API_DBG("%s: VL53L0_StaticInit Start\n", __func__);
	rc = VL53L0_StaticInit(ST_data_t);
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return rc;
	}

	if (rc == VL53L0_ERROR_NONE) {
        rc = VL53L0_SetVcselPulsePeriod(ST_data_t, 
                                VL53L0_VCSEL_PERIOD_PRE_RANGE, g_preRange);
   	}
	
    	if (rc == VL53L0_ERROR_NONE) {
        	rc = VL53L0_SetVcselPulsePeriod(ST_data_t, 
                                VL53L0_VCSEL_PERIOD_FINAL_RANGE, g_finalRange);
   	}
	VL53L0_GetVcselPulsePeriod(ST_data_t,VL53L0_VCSEL_PERIOD_PRE_RANGE,&g_preRange);
	VL53L0_GetVcselPulsePeriod(ST_data_t,VL53L0_VCSEL_PERIOD_FINAL_RANGE,&g_finalRange);	

	printk("PulsePeriod(%d, %d)\n",g_preRange,g_finalRange);


	VL53L0_GetLimitCheckValue(ST_data_t,VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,&signal);
	
	VL53L0_GetLimitCheckValue(ST_data_t,VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,&sigma);

	VL53L0_GetMeasurementTimingBudgetMicroSeconds(ST_data_t,&budget);

	printk("signal%d, sigma%d, budget%d\n",signal,sigma,budget);

	
	API_DBG("%s: VL53L0_StaticInit Success\n", __func__);
	return rc;
}
int Laser_whether_apply_calib(bool apply){
	int rc=0;
	if (laserSTx_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	
		Laser_device_off();

	rc = Laser_device_on();
	if(rc <0) return rc;

	rc = Laser_init_data();
	if(rc <0) return rc;

	rc = Laser_prepare();
	if(rc <0) return rc;

	if(apply){
		rc = VL6180x_device_Load_Calibration_Value();
		if (rc < 0){
			pr_err("%s Device trun on fail !!\n", __func__);
			laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			return rc;
		}
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
	}
	else
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;

	//temp
	if(timedMeasure){
		VL53L0_SetDeviceMode(ST_data_t,VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
		VL53L0_SetInterMeasurementPeriodMilliSeconds(ST_data_t,10000);
		ClosingLaser = false;
		printk("%s work here\n",__func__);
		queue_delayed_work(STMeasure_wq, &STMeasure_wk, 1*HZ);
	}
	
	printk("%s Init Device (%d), rc(%d)\n", __func__, laserSTx_t->device_state, rc);
	return rc;
}

int Laser_init_cci(void){
	int rc=0;
	//laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
	rc = dev_cci_init(laserSTx_t);
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return -EIO;
	} 

	rc = Laser_init_data();
	if(rc <0) return rc;

	rc = Laser_prepare();
	if(rc <0) return rc;

	rc = VL6180x_device_Load_Calibration_Value();
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return -EIO;
	}
		
	API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	rc = VL53L0_SetDeviceMode(ST_data_t, VL53L0_DEVICEMODE_SINGLE_RANGING);
	if (rc < 0){
		pr_err("%s Device trun on fail !!\n", __func__);
		laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		return -EIO;
	}
	API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	//laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
	printk("%s Init Device (%d)\n", __func__, laserSTx_t->device_state);

	camera_on_flag = true;
	log_count = 0;
	return rc;

}	
void Laser_deinit_cci(void){
	mutex_lock(&vl6180x_mutex);
	dev_cci_deinit(laserSTx_t);
	//laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
	camera_on_flag = false;
	mutex_unlock(&vl6180x_mutex);
}

static ssize_t ST_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	bool apply_calib=true;
	int val, rc = 0;
	char messages[8]="";
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (laserSTx_t->device_state == val)	{
		printk("%s Setting same commond (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:		
			if(camera_on_flag){
				CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			mutex_lock(&vl6180x_mutex);
			Laser_device_off();
			mutex_unlock(&vl6180x_mutex);
			break;
			
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:			
			if(camera_on_flag){
				CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			apply_calib = (val==MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
			rc = Laser_whether_apply_calib(apply_calib);
			if(rc < 0) goto DEVICE_TURN_ON_ERROR;	
			break;
				
		default:
			printk("%s commond fail !!\n", __func__);
			break;
	}
	printk("proc end\n");
	return len;

DEVICE_TURN_ON_ERROR:
	rc = dev_cci_deinit(laserSTx_t);
	if (rc < 0) {
		pr_err("%s dev_cci_deinit failed %d\n", __func__, __LINE__);
	}
		
	rc = PowerDown(laserSTx_t);
	if (rc < 0) {
		pr_err("%s PowerDown failed %d\n", __func__, __LINE__);
	}
	return -EIO;
}

int device_enable_open(void){
	int rc=0;

	printk("%s\n",__func__);
	rc = dev_cci_init(laserSTx_t);
	if (rc < 0) {
		pr_err("%s dev_cci_deinit failed %d\n", __func__, __LINE__);
	}
		
	rc = PowerUp(laserSTx_t);
	if (rc < 0) {
		pr_err("%s PowerDown failed %d\n", __func__, __LINE__);
		return -EIO;
	}
	return 0;
}	

static int ST_device_enable_read(struct seq_file *buf, void *v)
{
	printk("%s:%d\n",__func__ ,laserSTx_t->device_state);
	seq_printf(buf,"%s:%d\n",__func__ ,laserSTx_t->device_state);
	return 0;
}

static int ST_device_enable_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ST_device_enable_read, NULL);
}

const struct file_operations ST_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ST_device_enable_open,
	.write = ST_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int ST_device_read_range(VL53L0_RangingMeasurementData_t *pRangeData)
{
	int8_t status;
	int i = 0;
	uint8_t intStatus;
	int16_t RawRange;

	timer = get_current_time();
	log_count++;

	/* Setting Range meansurement in single-shot mode */	
	API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL53L0_ClearInterruptMask(ST_data_t,0);//????
	if (status < 0) {
		pr_err("%s: VL53L0_ClearInterruptMask failed\n", __func__);
		return (int)status;
	}
	API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL53L0_SetDeviceMode(ST_data_t, VL53L0_DEVICEMODE_SINGLE_RANGING);
	if (status < 0) {
		pr_err("%s: VL6180x_RangeSetSystemMode failed\n", __func__);
		return (int)status;
	}
	API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	RawRange = 0;

	/* Delay: waitting laser sensor to be triggered */
	msleep(8);

	/* Get Sensor detect distance */
	for (i = 0; i <1000; i++)	{
		/* Check RESULT_INTERRUPT_STATUS_GPIO */
		API_DBG("%s: VL53L0_ClearInterruptMask Start\n", __func__);
		status = VL53L0_ClearInterruptMask(ST_data_t, 0);
		if (status < 0) {
			pr_err("%s: VL53L0_ClearInterruptMask failed\n", __func__);
			return (int)status;
		}
		API_DBG("%s: VL53L0_ClearInterruptMask Success\n", __func__);

		if (intStatus == VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY){
			API_DBG("%s: VL53L0_GetRangingMeasurementData Start\n", __func__);
			status = VL53L0_GetRangingMeasurementData(ST_data_t, pRangeData);
			if (status < 0) {
				pr_err("%s: Read range failed\n", __func__);
				goto ERR_OUT_OF_RANGE;
			}
			if (pRangeData->RangeStatus == 0){
				if(log_count >= LOG_DISPLAY_COUNT){
					log_count = 0;
					DBG_LOG("%s: Read range:%d\n", __func__, (int)pRangeData->RangeMilliMeter);
				}
				API_DBG("%s: VL53L0_GetRangingMeasurementData Success\n", __func__);
				break;
			}
			else{
				API_DBG("%s: VL53L0_GetRangingMeasurementData Failed: errorStatus(%d)\n", __func__, pRangeData->RangeStatus);
				goto ERR_OUT_OF_RANGE;
			}
		}

		timer2 = get_current_time();
		if((((timer2.tv_sec*1000000)+timer2.tv_usec)-((timer.tv_sec*1000000)+timer.tv_usec)) > (TIMEOUT_VAL*1000)){
			printk("%s: Timeout: Out Of Range!!\n", __func__);
			goto ERR_OUT_OF_RANGE;
		}

		/* Delay: waitting laser sensor sample ready */
		msleep(5);
	}

	/* Setting SYSTEM_INTERRUPT_CLEAR to 0x01 */
	API_DBG("%s: VL53L0_ClearInterruptMask Start\n", __func__);
	status = VL53L0_ClearInterruptMask(ST_data_t,0);
	if (status < 0) {
		pr_err("%s: VL53L0_ClearInterruptMask failed\n", __func__);
		return (int)status;
	}
	API_DBG("%s: VL53L0_ClearInterruptMask Success\n", __func__);

	return (int)pRangeData->RangeMilliMeter;
ERR_OUT_OF_RANGE:
	/* Setting SYSTEM_INTERRUPT_CLEAR to 0x01 */
        API_DBG("%s: VL53L0_ClearInterruptMask Start\n", __func__);
        status = VL53L0_ClearInterruptMask(ST_data_t,0);
        if (status < 0) {
                pr_err("%s: VL53L0_ClearInterruptMask failed\n", __func__);
                return (int)status;
        }
        API_DBG("%s: VL53L0_ClearInterruptMask Success\n", __func__);

	return OUT_OF_RANGE;
}

void ST_keep_measure_work(struct work_struct *work){

	VL53L0_RangingMeasurementData_t pRangeData;
	uint8_t dataReady;
	int8_t status=0;

	do{	
		status = VL53L0_GetMeasurementDataReady(ST_data_t, &dataReady);
		if(status ==0)
			status = VL53L0_GetRangingMeasurementData(ST_data_t, &pRangeData);
		Range = pRangeData.RangeMilliMeter;
		ErrCode = pRangeData.RangeStatus;
		printk("WWW\n");
		msleep(50);
	}while(status==0 && !ClosingLaser);

	status = VL53L0_StopMeasurement(ST_data_t);
	printk("%s: stop (%d)",__func__,status);


	if(status < 0 && ClosingLaser)
		Laser_whether_apply_calib(laserSTx_t->device_state);
	//return status;
	
}



int read_range(VL53L0_RangingMeasurementData_t *pRangeData){
	int8_t status;
	log_count++;
	

if(timedMeasure){
	printk("timemeas\n");
	return Range;

}
else{
	status = VL53L0_ClearInterruptMask(ST_data_t, 0);
	if (status < 0) {
		pr_err("%s: VL53L0_ClearInterruptMask failed\n", __func__);
		return (int)status;
	}
	
	status = VL53L0_PerformSingleRangingMeasurement(ST_data_t, pRangeData);
	if (pRangeData->RangeStatus == 0){
		if(log_count >= LOG_DISPLAY_COUNT){
			log_count = 0;
			DBG_LOG("%s: Read range:%d\n", __func__, (int)pRangeData->RangeMilliMeter);
		}
		API_DBG("%s: VL53L0_GetRangingMeasurementData Success\n", __func__);
	}
	else{
		API_DBG("%s: VL53L0_GetRangingMeasurementData Failed: errorStatus(%d)\n", __func__, pRangeData->RangeStatus);
	}

	DMax = pRangeData->RangeDMaxMilliMeter;
	STerrorStatus = pRangeData->RangeStatus;
}

	if(status < 0) 
		return (int)status;
	else if(pRangeData->RangeStatus != 0){
		LOG_Handler(LOG_ERR,"%s: range error (%d)\n", __func__, pRangeData->RangeStatus);
		return -(OUT_OF_RANGE);
	}
	else 
		return (int)pRangeData->RangeMilliMeter;
}

bool device_invalid(void){
	return (laserSTx_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF);
}	

int check_range_read_method(void){
	
	if (device_invalid()) {
		LOG_Handler(LOG_ERR, "Device without turn on\n");
		return -EBUSY;
	}
	return	0;
}

int ReadRangeByProc(struct seq_file *buf, bool for_DIT){

	int RawRange = 0, rc=0;
	VL53L0_RangingMeasurementData_t RangeData;

	mutex_lock(&vl6180x_mutex);

	rc = check_range_read_method();	
	if(rc >=0 && for_DIT){
		RawRange = read_range(&RangeData);
		seq_printf(buf, "%d#%d#%d\n", RawRange, RangeData.RangeDMaxMilliMeter, RangeData.RangeStatus);
	}
	else if(rc >=0 && !for_DIT){
		RawRange = read_range(&RangeData);
		seq_printf(buf, "%d\n", RawRange);
	}
	else
		seq_printf(buf, "%d\n", 0);
	
	mutex_unlock(&vl6180x_mutex);
	return rc;
	
}

static int ST_device_get_range_read(struct seq_file *buf, void *v)
{
	return ReadRangeByProc(buf, 0);
}
 
static int ST_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ST_device_get_range_read, NULL);
}

const struct file_operations ST_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ST_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ST_device_get_range_more_info_read(struct seq_file *buf, void *v)
{
	return ReadRangeByProc(buf, 1);
}
 
static int ST_device_get_range_more_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ST_device_get_range_more_info_read, NULL);
}

const struct file_operations ST_get_range_more_info_fos = {
	.owner = THIS_MODULE,
	.open = ST_device_get_range_more_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int OffsetCalibration(void)
{
	int offset, status;

	if (device_invalid()) {
		pr_err("%s: Device without turn on (%d) \n", __func__,  laserSTx_t->device_state);
		return -EBUSY;
	}
		
		status = VL53L0_SetOffsetCalibrationDataMicroMeter(ST_data_t,0);
	if(status==0)
		status = VL53L0_SetXTalkCompensationEnable(ST_data_t,0);
	if(status==0)
		status = VL53L0_PerformOffsetCalibration(ST_data_t, (FixPoint1616_t)(VL6180_OFFSET_CAL_RANGE<<16), &offset);
	if(status==0)
		status = VL53L0_SetOffsetCalibrationDataMicroMeter(ST_data_t, offset);
	if (status < 0) {
		pr_err("%s: K fail (%d)\n", __func__, status);
		return status;
	}

	if (Laser_sysfs_write_offset(offset) == false){
		pr_err("%s: write fail\n", __func__);
		return -ENOENT;
	}

	DBG_LOG("%s: The offset value is %d um\n",__func__ ,offset);

	return 0;
}


int CrossTalkCalibration(void){

	uint8_t status=0;
	FixPoint1616_t XtalkCompRate;

	if (device_invalid()) {
		pr_err("%s: Device without turn on: (%d) \n", __func__, laserSTx_t->device_state);
		return -EBUSY;
	}
	
		status = VL53L0_SetXTalkCompensationRateMegaCps(ST_data_t,0);
	if(status==0)
		status = VL53L0_PerformXTalkCalibration(ST_data_t,(FixPoint1616_t) (VL6180_CROSSTALK_CAL_RANGE<<16), &XtalkCompRate);
	if(status < 0){
		pr_err("%s: K fail (%d)\n", __func__, status);
		return status;
	}


	if(Laser_sysfs_write_xtalk((int)XtalkCompRate) == false){
		pr_err("%s: write fail\n", __func__);
		return -ENOENT;
	}

	DBG_LOG("Crosstalk compensation rate is %u\n", XtalkCompRate);
	return 0;

}


int ReferenceCalibration(void){

	uint32_t pRefSpadCount;
	uint8_t pIsApertureSpads, pVhvSettings, pPhaseCal;
	uint32_t Refdata[REF_K_DATA_NUMBER];
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	
	Status = VL53L0_PerformRefSpadManagement(ST_data_t,&pRefSpadCount, &pIsApertureSpads);

	if(Status == VL53L0_ERROR_NONE)
		VL53L0_PerformRefCalibration(ST_data_t,&pVhvSettings, &pPhaseCal);

	Refdata[0] = pRefSpadCount;
	Refdata[1] = pIsApertureSpads;
	Refdata[2] = pVhvSettings;
	Refdata[3] = pPhaseCal;

	printk("%s: %x %x %x %x\n",__FUNCTION__ ,Refdata[0],Refdata[1],Refdata[2],Refdata[3]);

	//to do write data to file
	Sysfs_write_Dword_seq(LASER_K_REF_FILE, Refdata, REF_K_DATA_NUMBER);
	
	Refdata[0] = 123456;
	Refdata[1] = 10;
	Refdata[2] = 20;
	Refdata[3] = 30;
	Sysfs_read_Dword_seq(LASER_K_REF_FILE, Refdata, 4);
	printk("%s: %x %x %x %x\n",__FUNCTION__ ,Refdata[0],Refdata[1],Refdata[2],Refdata[3]);

	return (int)Status;
}

int DmaxCalibration(void){

	VL53L0_RangingMeasurementData_t data;
	FixPoint1616_t  RtnRate, errStatus;
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	FixPoint1616_t Dmax[DMAX_K_DATA_NUMBER];
	Status = VL53L0_PerformSingleRangingMeasurement(ST_data_t, &data);

	//Dist = (VL6180_CROSSTALK_CAL_RANGE<<16);
	RtnRate = data.SignalRateRtnMegaCps;
	errStatus	= data.RangeStatus;

	printk("%s: RtnRate %x\n",__FUNCTION__ , RtnRate);

	Dmax[0] = data.RangeDMaxMilliMeter;
	Dmax[1] = RtnRate;
	Dmax[2] = errStatus;
	//write to data
	Sysfs_write_Dword_seq(LASER_K_DMAX_FILE,Dmax, DMAX_K_DATA_NUMBER);
	
	printk("%s: %x %x %x",__FUNCTION__ , Dmax[0], Dmax[1], Dmax[2]);
	return Status;
}
 
static ssize_t ST_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8]="";
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	printk("%s commond : %d\n", __func__, val);
	switch (val) {
	case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		ret = ReferenceCalibration();
		if(ret == 0)
		ret = OffsetCalibration();
		if (ret < 0)
			return ret;
		break;
	case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		ret = CrossTalkCalibration();
		if(ret == 0)
		ret = DmaxCalibration();
		if (ret < 0)
			return ret;
		break;
	default:
		printk("%s commond fail(%d) !!\n", __func__, val);
		return -EINVAL;
	}
	return len;
}

const struct file_operations ST_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ST_device_get_range_open,
	.write = ST_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



static int ST_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = ST_I2C_status_check(laserSTx_t,MSM_CAMERA_I2C_BYTE_DATA);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int ST_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ST_I2C_status_check_proc_read, NULL);
}

const struct file_operations ST_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ST_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int VL6180x_I2C_status_check_via_prob(struct msm_laser_focus_ctrl_t *s_ctrl){
	return vl6180x_check_status;
}

int VL6180x_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = VL6180x_I2C_status_check_via_prob(laserSTx_t);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

int VL6180x_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, VL6180x_I2C_status_check_proc_read, NULL);
}

const struct file_operations I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = VL6180x_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_VL6180x_register_read(struct seq_file *buf, void *v)
{
	int status, i = 0;
	uint16_t register_value = 0;

	for (i = 0; i <0x100; i++)	{
		register_value = 0;
		status = ST_CCI_RdWord(i, &register_value);
		printk("%s: read register(0x%x): 0x%x for word\n",__func__, i, register_value);
		if (status < 0) {
			pr_err("%s: read register(0x%x) failed\n", __func__, i);
			return status;
		}
	}
	seq_printf(buf, "%d\n", 0);
	return 0;
}

static int dump_VL6180x_laser_focus_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_VL6180x_register_read, NULL);
}

const struct file_operations dump_ST_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_VL6180x_laser_focus_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_VL6180x_debug_register_read(struct seq_file *buf, void *v)
{
	uint16_t reg_data = 0;

	mutex_lock(&vl6180x_mutex);
	
	if (device_invalid()) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, laserSTx_t->device_state);
		mutex_unlock(&vl6180x_mutex);
		return -EBUSY;
	}
	
	ST_CCI_RdWord(VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, reg_data);
	
	ST_CCI_RdByte(VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, reg_data);
	
	ST_CCI_RdByte(VL53L0_REG_RESULT_RANGE_STATUS, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", VL53L0_REG_RESULT_RANGE_STATUS, reg_data);
/*	
	ST_CCI_RdWord(VL53L0_REG_RESULT_SIGNAL_COUNT_RATE_RET, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", VL53L0_REG_RESULT_SIGNAL_COUNT_RATE_RET, reg_data);
	VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;//?
*/
	seq_printf(buf, "DMax : %d\n", DMax);
	seq_printf(buf, "errorStatus : %d\n", STerrorStatus);

	mutex_unlock(&vl6180x_mutex);

	return 0;
}

static int dump_VL6180x_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_VL6180x_debug_register_read, NULL);
}

const struct file_operations dump_ST_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_VL6180x_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
//back door
int dump_ST_value_check_read(struct seq_file *buf, void *v)
{
	timedMeasure = !timedMeasure;
	printk("timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf("timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf(buf,"PASS\n");
       return 0;
}

int dump_ST_value_check_open(struct inode *inode, struct  file *file)
{
        return single_open(file, dump_ST_value_check_read, NULL);
}

const struct file_operations dump_ST_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_ST_value_check_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

*/

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops; 

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = "qcom,ois", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);

static int matchST(struct platform_device *pdev){

	const struct of_device_id *match;

	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if(!match){
		LOG_Handler(LOG_ERR, "device not match\n");
		return -EFAULT;
	}

	if(!pdev->dev.of_node){
		LOG_Handler(LOG_ERR, "of_node NULL\n");
		return -EINVAL;
	}
	
	laserSTx_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laserSTx_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	laserSTx_t->pdev = pdev;

	ST_data_t = kzalloc(sizeof(VL53L0_Dev_t),GFP_KERNEL);
	if (!ST_data_t) {
		pr_err("%s:%d failed no memory for ST_data_t\n", __func__, __LINE__);
		return -ENOMEM;
	}
	
	return 0;
}

static int32_t laser_ST_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;

	rc = matchST(pdev);
		
	if(rc==0)
	rc = get_dtsi_data(pdev->dev.of_node, laserSTx_t);
	
	if(rc==0)
	rc = set_i2c_client(laserSTx_t, &msm_laser_focus_i2c_client);

	if(rc<0)
		goto probe_failure;
	
	set_cci_client(laserSTx_t);
	set_subdev(laserSTx_t, &msm_laser_focus_internal_ops);
	set_laser_state(laserSTx_t);
	
	vl6180x_check_status = ST_I2C_status_check(laserSTx_t, MSM_CAMERA_I2C_BYTE_DATA);
	if(vl6180x_check_status==I2C_STATUS_FAIL)
		goto probe_failure;

	ST_create_proc_file();
	ATD_status = 1;
	
	STMeasure_wq = create_singlethread_workqueue("Laser_wq");
	INIT_DELAYED_WORK(&STMeasure_wk, ST_keep_measure_work);
	
	g_factory = g_ftm_mode;
	if(g_factory){
		Enable_DBG();
		timedMeasure = false;
	}	
	LOG_Handler(LOG_CDBG, "%s: timedMeasure(%d), factory_mode(%d)\n", __func__,timedMeasure, g_factory);

	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
	
probe_failure:
	LOG_Handler(LOG_ERR, "%s: Probe failed, rc = %d\n", __func__, rc);
	return rc;
}

static struct platform_driver msm_laser_focus_platform_driver = {
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int __init Laser_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	mutex_init(&vl6180x_mutex);
	rc = platform_driver_probe(&msm_laser_focus_platform_driver,
		laser_ST_platform_probe);
	DBG_LOG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return rc;
}
static void __exit Laser_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	return;
}

#if 0	//these will be deleted
static int32_t PowerUp(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = VL6180x_vreg_control(a_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	a_ctrl->laser_focus_state = LASER_FOCUS_POWER_UP;

	VL6180x_GPIO_High(a_ctrl);

	CDBG("Exit\n");
	return rc;
}

static int32_t PowerDown(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->laser_focus_state != LASER_FOCUS_POWER_DOWN) {

		rc = VL6180x_vreg_control(a_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}

	VL6180x_GPIO_Low(a_ctrl);
	
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_init(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	
	// CCI Init 
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_deinit(struct msm_laser_focus_ctrl_t *a_ctrl) 
{
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
		
	}


	CDBG("Exit\n");
	return rc;
}


int VL6180x_match_id(struct msm_laser_focus_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	DBG_LOG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);
	if (chipid != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static int VL6180x_GPIO_High(struct msm_laser_focus_ctrl_t *a_ctrl){
	int rc = 0;
	
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	CDBG("Enter\n");

	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	
	sensordata = a_ctrl->sensordata;
	power_info = &sensordata->power_info;

	if(power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL){
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if(rc < 0){
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VDIG],
		GPIO_OUT_HIGH
	);
	
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_GPIO_Low(struct msm_laser_focus_ctrl_t *a_ctrl){
	int rc = 0;
	
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	CDBG("Enter\n");

	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	
	sensordata = a_ctrl->sensordata;
	power_info = &sensordata->power_info;

	if(power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL){
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VDIG],
		GPIO_OUT_LOW
	);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if(rc < 0){
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	
	CDBG("Exit\n");
	return rc;
}


static int32_t VL6180x_vreg_control(struct msm_laser_focus_ctrl_t *a_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_laser_focus_vreg *vreg_cfg;

	vreg_cfg = &a_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(a_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}


static int32_t VL6180x_get_dt_data(struct device_node *of_node,
		struct msm_laser_focus_ctrl_t *fctrl)
{
	int i = 0;
	int32_t rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint32_t id_info[3];
	struct msm_laser_focus_vreg *vreg_cfg = NULL;
	
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->sensordata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = fctrl->sensordata;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	DBG_LOG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&sensordata->sensor_name);
	DBG_LOG("%s label %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_master);
	DBG_LOG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_master = MASTER_0;
		rc = 0;
	}

	if (of_find_property(of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &fctrl->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(fctrl);
			pr_err("failed rc %d\n", rc);
			return rc;
		}
	}

	sensordata->slave_info =
		kzalloc(sizeof(struct msm_camera_slave_info),
			GFP_KERNEL);
	if (!sensordata->slave_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}

	rc = of_property_read_u32_array(of_node, "qcom,slave-id",
		id_info, 3);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	fctrl->sensordata->slave_info->sensor_slave_addr = id_info[0];
	fctrl->sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	fctrl->sensordata->slave_info->sensor_id = id_info[2];

		CDBG("%s:%d slave addr 0x%x sensor reg 0x%x id 0x%x\n",
		__func__, __LINE__,
		fctrl->sensordata->slave_info->sensor_slave_addr,
		fctrl->sensordata->slave_info->sensor_id_reg_addr,
		fctrl->sensordata->slave_info->sensor_id);

	/* Handle GPIO (CAM_1V2_EN) */
	power_info = &sensordata->power_info;
	
	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if(!power_info->gpio_conf){
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	gconf = power_info->gpio_conf;
	
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if(gpio_array_size){
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
		if(!gpio_array){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf);
			rc = -ENOMEM;
			goto ERROR;
		}
		
		for(i=0; i < gpio_array_size; i++){
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i, gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf);
			goto ERROR;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_req_tbl);
			goto ERROR;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_set_tbl);
			goto ERROR;
		}
	}
	kfree(gpio_array);

	return rc;

ERROR:
		kfree(fctrl->sensordata->slave_info);
	return rc;
}



static int set_i2c_client(void){

	/* Assign name for sub device */
	snprintf(laserSTx_t->msm_sd.sd.name, sizeof(laserSTx_t->msm_sd.sd.name),
			"%s", laserSTx_t->sensordata->sensor_name);

	laserSTx_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	laserSTx_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	laserSTx_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == laserSTx_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		return -EFAULT;

	}
	if (!laserSTx_t->i2c_client->i2c_func_tbl)
		laserSTx_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	laserSTx_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!laserSTx_t->i2c_client->cci_client) {
		kfree(laserSTx_t->vreg_cfg.cam_vreg);
		kfree(laserSTx_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	return 0;	

}

static int set_cci_client(void){

	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = laserSTx_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = laserSTx_t->cci_master;
	if (laserSTx_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = laserSTx_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

	return 0;
}

static int set_subdev(void){

	v4l2_subdev_init(&laserSTx_t->msm_sd.sd, laserSTx_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&laserSTx_t->msm_sd.sd, laserSTx_t);
	
	laserSTx_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	laserSTx_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(laserSTx_t->msm_sd.sd.name,
		ARRAY_SIZE(laserSTx_t->msm_sd.sd.name), "msm_laser_focus");
	
	media_entity_init(&laserSTx_t->msm_sd.sd.entity, 0, NULL, 0);
	laserSTx_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	//laserSTx_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	laserSTx_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&laserSTx_t->msm_sd);

	return 0;
}


static int set_laser_config(void){

	/* Init data struct */
	laserSTx_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	laserSTx_t->laser_focus_cross_talk_offset_value = 0;
	//laserSTx_t->laser_focus_offset_value = 0;
	laserSTx_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;

	return 0;
}


static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
	//.s_power = msm_laser_focus_power,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};


static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

#endif

module_init(Laser_init_module);
module_exit(Laser_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
