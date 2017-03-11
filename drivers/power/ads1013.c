/*
 * ads1015.c - lm_sensors driver for ads1015 12-bit 4-input ADC
 * (C) Copyright 2010
 * Dirk Eibach, Guntermann & Drunck GmbH <eibach@gdsys.de>
 *
 * Based on the ads7828 driver by Steve Hardy.
 *
 * Datasheet available at: http://focus.ti.com/lit/ds/symlink/ads1015.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/gpio.h>


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/spmi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

#include <linux/i2c/ads1015.h>

//ASUS BSP Austin_T : global ADS1013_READY +++
bool ads1013_ready;
EXPORT_SYMBOL(ads1013_ready);

//Define register addresses of ads1013 0X48
#define ads1013_raddr 0x48

struct ads1013_data
{
	u32 gpio_134;
	u32 gpio_flags134;
};

struct i2c_client *ads1013_client;

/*
ads1013_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/
int ads1013_write_reg(uint8_t slave_addr, uint8_t cmd_reg, uint16_t write_val)
{
	int ret = 0;

	ads1013_client->addr = slave_addr; //real SMBus address (8 bits)
	ret = i2c_smbus_write_word_data(ads1013_client, cmd_reg, write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);
	}
	return ret;
}

/*
ads1013_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
store_read_val  :	value be read will store here

*/
int ads1013_read_reg(uint8_t slave_addr, uint8_t cmd_reg, uint16_t *store_read_val)
{
	int ret = 0;
	u8 buf[2];
	u16 val;

	ads1013_client->addr = slave_addr;
	printk("[BAT][CHG] ads1013_read_reg addr=%x\n", slave_addr);
	ret = i2c_smbus_read_i2c_block_data(ads1013_client, cmd_reg, 2, buf);
	printk("[BAT][CHG] buf[0] = 0x%x\n", buf[0]);
	printk("[BAT][CHG] buf[1] = 0x%x\n", buf[1]);
	val = (buf[0] << 8) + buf[1];
	printk("[BAT][CHG] ret=%d\n", ret);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, slave_addr);
	}

	*store_read_val = (uint16_t) val;

	return 0;
}

u16 ads1013_dump_value(void)
{
	u16 my_read_value = 0;

	ads1013_write_reg(ads1013_raddr, 0x01, 0x8583);
	ads1013_read_reg(ads1013_raddr, 0x00, &my_read_value);
	printk("[BAT][CHG] AIN0 voltage value = 0x%xh\n", my_read_value);

	return my_read_value;
}
EXPORT_SYMBOL(ads1013_dump_value);

static ssize_t adc_reg_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 ret;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);
	ret = ads1013_dump_value();

	return sprintf(buf, "reg value = 0x%xh\n", ret);
}

static DEVICE_ATTR(adc_reg_value, 0664, adc_reg_value_show, NULL);

static struct attribute *dump_reg_attrs[] = {
	&dev_attr_adc_reg_value.attr,
	NULL
};

static const struct attribute_group dump_reg_attr_group = {
	.attrs = dump_reg_attrs,
};

static int ads1013_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ads1013_data *data;
	int rc;
	int ret;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);

	ads1013_ready = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[BAT][CHG] %s: i2c bus does not support the ads1013\n", __FUNCTION__);
	}

	data = devm_kzalloc(&client->dev, sizeof(struct ads1013_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	ads1013_client = client;
	i2c_set_clientdata(client, data);

	ret = ads1013_write_reg(ads1013_raddr, 0x01, 0x0);
	if (ret < 0) {
		printk("[BAT][CHG] %s: i2c slave_addr 0x48 for ads1013 not ACK\n", __FUNCTION__);
		return -ENODEV;
	}

	rc = sysfs_create_group(&client->dev.kobj, &dump_reg_attr_group);
	if (rc)
		goto exit_remove;

	ads1013_ready = 1;

	printk("[BAT][CHG] %s end\n", __FUNCTION__);

	return 0;

exit_remove:
		sysfs_remove_group(&client->dev.kobj, &dump_reg_attr_group);
	return rc;

}

static struct of_device_id ads1013_match_table[] = {
	{ .compatible = "qcom,ads1013",},
	{ },
};

static const struct i2c_device_id ads1013_id[] = {
	{ "ads1013", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1013_id);

static struct i2c_driver ads1013_driver = {
	.driver = {
		.name = "ads1013",
		.owner		= THIS_MODULE,
		.of_match_table	= ads1013_match_table,
	},
	.probe = ads1013_probe,
	.id_table = ads1013_id,
};

module_i2c_driver(ads1013_driver);

MODULE_AUTHOR("Dirk Eibach <eibach@gdsys.de>");
MODULE_DESCRIPTION("ADS1013 driver");
MODULE_LICENSE("GPL");
