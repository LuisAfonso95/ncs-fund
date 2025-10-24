/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */

#if defined(CONFIG_BMP280)
#define CHIP_ID  0x58
#else
#define CHIP_ID  0x60
#endif

#define SENSOR_CONFIG_VALUE 0x93
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define ID	     0xD0
#define TEMPMSB	 0xFA
#define HUM_ADDR 0xFD
#define HUM_DIG_ADDR1	 0xA1
#define HUM_DIG_ADDR2	 0xE1

/* STEP 6 - Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysensor)

/* Data structure to store BME280 data */
struct bme280_data {
	/* Compensation parameters */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint8_t   dig_H1;
	int16_t dig_H2;
	int8_t   dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t   dig_H6 ;
} bmedata;


void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	/* Step 10 - Put calibration function code */
	uint8_t values[8];

	int ret = i2c_burst_read_dt(spec, CALIB00, values, 6);

	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}
	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];

	ret = i2c_burst_read_dt(spec, HUM_DIG_ADDR1, values, 1);
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}

	ret = i2c_burst_read_dt(spec, HUM_DIG_ADDR2, &values[1], 7);
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}
   sensor_data_ptr->dig_H1 =   values[0];
   sensor_data_ptr->dig_H2 = (values[2] << 8) | values[1];
   sensor_data_ptr->dig_H3 =   values[3];
   sensor_data_ptr->dig_H4 = ((int8_t)values[4] * 16) | (0x0F & values[5]);
   sensor_data_ptr->dig_H5 = ((int8_t)values[6] * 16) | ((values[5] >> 4) & 0x0F);
   sensor_data_ptr->dig_H6 =   values[7];
	
}

/* Compensate current temperature using previously stored sensor calibration data */
static float bme280_calculate_humidity(struct bme280_data *data, int32_t adc_humidity, int32_t t_fine)
{
   // Code based on calibration algorthim provided by Bosch.
   int32_t var1;


   var1 = (t_fine - ((int32_t)76800));
   var1 = (((((adc_humidity << 14) - (((int32_t)data->dig_H4) << 20) - (((int32_t)data->dig_H5) * var1)) +
   ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)data->dig_H6)) >> 10) * (((var1 *
   ((int32_t)data->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
   ((int32_t)data->dig_H2) + 8192) >> 14));
   var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)data->dig_H1)) >> 4));
   var1 = (var1 < 0 ? 0 : var1);
   var1 = (var1 > 419430400 ? 419430400 : var1);
   return ((uint32_t)(var1 >> 12))/1024.0;
}

static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp, int32_t *t_fine_p)
{
	int32_t var1, var2;
	int32_t t_fine;


	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;
	t_fine = (var1 + var2);
	if(t_fine_p != NULL)
	{
		*t_fine_p = t_fine;
	}


	return ((t_fine) * 5 + 128) >> 8;
}

int main(void)
{

	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}
	/* STEP 9 - Verify it is proper device by reading device id  */
	  uint8_t id = 0;
	uint8_t regs[] = {ID};

	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);

	if (ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
		return -1;
	}
	printk("Expecting id 0x%x\n", CHIP_ID);
	if (id != CHIP_ID) {
		printk("Invalid chip id! %x \n", id);
		return -1;
	}

	bme_calibrationdata(&dev_i2c, &bmedata);

	/* STEP 11 - Setup the sensor by writing the value 0x93 to the Configuration register */
	uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};

	
	ret = i2c_write_dt(&dev_i2c, sensor_config, 2);

	if (ret != 0) {
		printk("Failed to write register %x \n", sensor_config[0]);
		return -1;
	}
	while (1) {

		/* STEP 12 - Read the temperature from the sensor */
		uint8_t temp_val[3]= {0};

		int ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, 3);

		if (ret != 0) {
			printk("Failed to read register %x \n", TEMPMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}
		/* STEP 12.1 - Put the data read from registers into actual order (see datasheet) */
		int32_t adc_temp =
			(temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);
		int32_t t_fine;

		/* STEP 12.2 - Compensate temperature */
		int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp, &t_fine);
		/* STEP 12.3 - Convert temperature */
		float temperature = (float)comp_temp / 100.0f;
		double fTemp = (double)temperature * 1.8 + 32;

		// Print reading to console
		printk("Temperature in Celsius : %8.2f C\n", (double)temperature);
		printk("Temperature in Fahrenheit : %.2f F\n", fTemp);


		uint8_t hum_val[2]= {0};
		ret = i2c_burst_read_dt(&dev_i2c, HUM_ADDR, hum_val, 2);
		if (ret != 0) {
			printk("Failed to read register %x \n", TEMPMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}
		uint32_t rawHumidity = (hum_val[0] << 8) | hum_val[1];
		printk("Raw Humidity : 0X%x\n", rawHumidity);
		
		float humidity=  bme280_calculate_humidity(&bmedata,  rawHumidity, t_fine);
		printk("Humidity : %f\n", (double)humidity);
		
		

		k_msleep(SLEEP_TIME_MS);
	}
}
