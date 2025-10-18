#include <lvgl.h>
#include <lvgl_input_device.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "logo_image.h"

#define BASIC_TEST

#define DISPLAY_BUFFER_PITCH 128

LOG_MODULE_REGISTER(display);

#ifdef BASIC_TEST
static uint32_t count;

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static struct gpio_dt_spec btn0_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback btn0_callback;

static void btn01_isr_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    count = 0;
}

static void lv_btn_click_callback(lv_event_t *e)
{
    ARG_UNUSED(e);

    count = 0;
}

#define CHIP_ID  0x58
#define SENSOR_CONFIG_VALUE 0x93
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define ID	     0xD0
#define TEMPMSB	 0xFA

#define I2C_NODE DT_NODELABEL(mysensor)

/* Data structure to store BME280 data */
struct bme280_data {
	/* Compensation parameters */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
} bmedata;


void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	/* Step 10 - Put calibration function code */
	uint8_t values[6];

	int ret = i2c_burst_read_dt(spec, CALIB00, values, 6);

	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}
	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];
	
}

/* Compensate current temperature using previously stored sensor calibration data */
static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;

	return ((var1 + var2) * 5 + 128) >> 8;
}
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
static int init_sensor()
{
	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */

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
}

static float get_temp()
{
	uint8_t temp_val[3] = {0};

	int ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, 3);

	if (ret != 0)
	{
		printk("Failed to read register %x \n", TEMPMSB);
		k_msleep(100);
		return 0.0;
	}
	/* STEP 12.1 - Put the data read from registers into actual order (see datasheet) */
	int32_t adc_temp =
		(temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);
	/* STEP 12.2 - Compensate temperature */
	int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
	/* STEP 12.3 - Convert temperature */
	float temperature = (float)comp_temp / 100.0f;

	return temperature;
}
int main(void)
{
	init_sensor();
    if (display == NULL)
    {
        LOG_ERR("device pointer is NULL");
        return 0;
    }

    if (!device_is_ready(display))
    {
        LOG_ERR("display device is not ready");
        return 0;
    }

    if (gpio_is_ready_dt(&btn0_gpio))
    {
        int err;

        err = gpio_pin_configure_dt(&btn0_gpio, GPIO_INPUT);
        if (err)
        {
            LOG_ERR("failed to configure button gpio: %d", err);
            return 0;
        }

        gpio_init_callback(&btn0_callback, btn01_isr_callback, BIT(btn0_gpio.pin));

        err = gpio_add_callback(btn0_gpio.port, &btn0_callback);
        if (err)
        {
            LOG_ERR("failed to add button callback: %d", err);
            return 0;
        }

        err = gpio_pin_interrupt_configure_dt(&btn0_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (err)
        {
            LOG_ERR("failed to enable button callback: %d", err);
            return 0;
        }
    }

    struct display_capabilities capabilities;
    display_get_capabilities(display, &capabilities);

    const uint16_t x_res = capabilities.x_resolution;
    const uint16_t y_res = capabilities.y_resolution;

    LOG_INF("x_resolution: %d", x_res);
    LOG_INF("y_resolution: %d", y_res);
    LOG_INF("supported pixel formats: %d", capabilities.supported_pixel_formats);
    LOG_INF("screen_info: %d", capabilities.screen_info);
    LOG_INF("current_pixel_format: %d", capabilities.current_pixel_format);
    LOG_INF("current_orientation: %d", capabilities.current_orientation);

    display_blanking_off(display);

    const struct display_buffer_descriptor buf_desc = {
        .width = x_res, .height = y_res, .buf_size = x_res * y_res, .pitch = DISPLAY_BUFFER_PITCH};

    if (display_write(display, 0, 0, &buf_desc, logo_buf) != 0)
    {
        LOG_ERR("could not write to display");
    }

    if (display_set_contrast(display, 0) != 0)
    {
        LOG_ERR("could not set display contrast");
    }

    const size_t ms_sleep = 5;

    // Increase brightness
    for (size_t i = 0; i < 255; i++)
    {
        display_set_contrast(display, i);
        k_sleep(K_MSEC(ms_sleep));
    }

    // Decrease brightness
    for (size_t i = 255; i > 0; i--)
    {
        display_set_contrast(display, i);
        k_sleep(K_MSEC(ms_sleep));
    }

    display_blanking_on(display);

    char count_str[11] = {0};
    lv_obj_t *hello_world_label;
    lv_obj_t *count_label;

    if (IS_ENABLED(CONFIG_LV_Z_POINTER_INPUT))
    {
        lv_obj_t *hello_world_button;

        hello_world_button = lv_btn_create(lv_scr_act());
        lv_obj_align(hello_world_button, LV_ALIGN_CENTER, 0, -15);
        lv_obj_add_event_cb(hello_world_button, lv_btn_click_callback, LV_EVENT_CLICKED, NULL);
        hello_world_label = lv_label_create(hello_world_button);
    }
    else
    {
        hello_world_label = lv_label_create(lv_scr_act());
    }

    lv_label_set_text(hello_world_label, "Temperature");
    lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

    count_label = lv_label_create(lv_scr_act());
    lv_obj_align(count_label, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_task_handler();
    display_blanking_off(display);

    while (true)
    {

		float temperature = get_temp();
		char temp_str[] = "-1000.00 C";
		sprintf(temp_str, "%8.2f C", temperature);
		lv_label_set_text(count_label, temp_str);
		lv_task_handler();
        k_sleep(K_MSEC(1000));
    }
}

#else

#endif