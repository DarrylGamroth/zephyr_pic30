/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include "adxl355.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(ADXL355, CONFIG_SENSOR_LOG_LEVEL);

static void adxl355_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct adxl355_data *drv_data = dev->driver_data;
	const struct adxl355_dev_config *cfg = dev->config->config_info;
	u8_t status1, status2;

	/* Clear the status */
	if (adxl355_get_status(dev, &status1, &status2, NULL) < 0) {
		return;
	}

	if (drv_data->th_handler != NULL) {
		/* In max peak mode we wait until we settle below the inactivity
		 * threshold and then call the trigger handler.
		 */
		if (cfg->max_peak_detect_mode &&
			ADXL355_STATUS_2_INACT(status2)) {
			drv_data->th_handler(dev, &drv_data->th_trigger);
		} else if (!cfg->max_peak_detect_mode &&
			(ADXL355_STATUS_2_INACT(status2) ||
			ADXL355_STATUS_2_ACTIVITY(status2))) {
			drv_data->th_handler(dev, &drv_data->th_trigger);
		}
	}

	if ((drv_data->drdy_handler != NULL) &&
		ADXL355_STATUS_1_DATA_RDY(status1)) {
		drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_gpio,
				     GPIO_INT_EDGE_TO_ACTIVE);
}

static void adxl355_gpio_callback(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	struct adxl355_data *drv_data =
		CONTAINER_OF(cb, struct adxl355_data, gpio_cb);
	const struct adxl355_dev_config *cfg = dev->config->config_info;

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_gpio,
				     GPIO_INT_DISABLE);

#if defined(CONFIG_ADXL355_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ADXL355_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_ADXL355_TRIGGER_OWN_THREAD)
static void adxl355_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct adxl355_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		adxl355_thread_cb(dev);
	}
}

#elif defined(CONFIG_ADXL355_TRIGGER_GLOBAL_THREAD)
static void adxl355_work_cb(struct k_work *work)
{
	struct adxl355_data *drv_data =
		CONTAINER_OF(work, struct adxl355_data, work);

	adxl355_thread_cb(drv_data->dev);
}
#endif

int adxl355_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct adxl355_data *drv_data = dev->driver_data;
	const struct adxl355_dev_config *cfg = dev->config->config_info;
	u8_t int_mask, int_en, status1, status2;
	int ret;

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_gpio,
				     GPIO_INT_DISABLE);

	switch (trig->type) {
	case SENSOR_TRIG_THRESHOLD:
		drv_data->th_handler = handler;
		drv_data->th_trigger = *trig;
		int_mask = ADXL355_INT1_MAP_ACT_MSK |
			   ADXL355_INT1_MAP_INACT_MSK;
		break;
	case SENSOR_TRIG_DATA_READY:
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = *trig;
		int_mask = ADXL355_INT1_MAP_DATA_RDY_MSK;
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		ret = -ENOTSUP;
		goto out;
	}

	if (handler) {
		int_en = int_mask;
	} else {
		int_en = 0U;
	}

	ret = adxl355_reg_write_mask(dev, ADXL355_INT1_MAP, int_mask, int_en);

	adxl355_get_status(dev, &status1, &status2, NULL); /* Clear status */
out:
	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_gpio,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return ret;
}

int adxl355_init_interrupt(struct device *dev)
{
	struct adxl355_data *drv_data = dev->driver_data;
	const struct adxl355_dev_config *cfg = dev->config->config_info;

	drv_data->gpio = device_get_binding(cfg->gpio_port);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
		    cfg->gpio_port);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, cfg->int_gpio,
			   GPIO_INPUT | cfg->int_flags);

	gpio_init_callback(&drv_data->gpio_cb,
			   adxl355_gpio_callback,
			   BIT(cfg->int_gpio));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}

#if defined(CONFIG_ADXL355_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ADXL355_THREAD_STACK_SIZE,
			(k_thread_entry_t)adxl355_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_ADXL355_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ADXL355_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = adxl355_work_cb;
	drv_data->dev = dev;
#endif

	return 0;
}
