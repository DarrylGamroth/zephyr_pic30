/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADXL355_ADXL355_H_
#define ZEPHYR_DRIVERS_SENSOR_ADXL355_ADXL355_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <sys/util.h>

/*
 * ADXL355 registers definition
 */
#define ADXL355_DEVID		0x00u  /* Analog Devices accelerometer ID */
#define ADXL355_DEVID_MST	0x01u  /* Analog Devices MEMS device ID */
#define ADXL355_PARTID		0x02u  /* Device ID */
#define ADXL355_REVID		0x03u  /* Product revision ID */
#define ADXL355_STATUS  	0x04u  /* Status register */
#define ADXL355_FIFO_ENTRIES	0x05u  /* Valid data samples in the FIFO */
#define ADXL355_TEMP2   	0x06u  /* Temperature data [11:8] */
#define ADXL355_TEMP1   	0x07u  /* Temperature data [7:0] */
#define ADXL355_X_DATA2 	0x08u  /* X-axis acceleration data [19:12] */
#define ADXL355_X_DATA1 	0x09u  /* X-axis acceleration data [11:4] */
#define ADXL355_X_DATA0 	0x0Au  /* X-axis acceleration data [3:0] */
#define ADXL355_Y_DATA2 	0x0Bu  /* Y-axis acceleration data [19:12] */
#define ADXL355_Y_DATA1 	0x0Cu  /* Y-axis acceleration data [11:4] */
#define ADXL355_Y_DATA0 	0x0Du  /* Y-axis acceleration data [3:0] */
#define ADXL355_Z_DATA2 	0x0Eu  /* Z-axis acceleration data [19:12] */
#define ADXL355_Z_DATA1 	0x0Fu  /* Z-axis acceleration data [11:4] */
#define ADXL355_Z_DATA0 	0x10u  /* Z-axis acceleration data [3:0] */
#define ADXL355_FIFO_DATA	0x11u  /* FIFO Data */
#define ADXL355_OFFSET_X_H	0x1Eu  /* X axis offset data [15:8] */
#define ADXL355_OFFSET_X_L	0x1Fu  /* X axis offset data [7:0] */
#define ADXL355_OFFSET_Y_H	0x20u  /* Y axis offset data [15:8] */
#define ADXL355_OFFSET_Y_L	0x21u  /* Y axis offset data [7:0] */
#define ADXL355_OFFSET_Z_H	0x22u  /* Z axis offset data [15:8] */
#define ADXL355_OFFSET_Z_L	0x23u  /* Z axis offset data [7:0] */
#define ADXL355_ACT_EN  	0x24u  /* Activity Enable Register */
#define ADXL355_ACT_THRESH_H	0x25u  /* Activity Threshold [15:8] */
#define ADXL355_ACT_THRESH_L	0x26u  /* Activity Threshold [7:0] */
#define ADXL355_ACT_COUNT	0x27u  /* Activity Count */
#define ADXL355_FILTER		0x28u  /* Filter Settings */
#define ADXL355_FIFO_SAMPLES	0x29u  /* FIFO Samples */
#define ADXL355_INT_MAP 	0x2Au  /* Interrupt mapping control */
#define ADXL355_SYNC		0x2Bu  /* Sync */
#define ADXL355_RANGE		0x2Cu  /* Range */
#define ADXL355_POWER_CTL	0x2Du  /* Power control */
#define ADXL355_SELF_TEST	0x2Eu  /* Self Test */
#define ADXL355_RESET		0x2Fu  /* Reset */

#define ADXL355_DEVID_VAL	0xADu  /* Analog Devices accelerometer ID */
#define ADXL355_MST_DEVID_VAL	0x1Du  /* Analog Devices MEMS device ID */
#define ADXL355_PARTID_VAL	0xEDu  /* Device ID */
#define ADXL355_REVID_VAL	0x01u  /* product revision ID*/
#define ADXL355_RESET_CODE	0x52u  /* Writing code 0x52 resets the device */

#define ADXL355_READ		0x01u
#define ADXL355_REG_READ(x)	(((x & 0xFF) << 1) | ADXL355_READ)
#define ADXL355_REG_WRITE(x)	((x & 0xFF) << 1)
#define ADXL355_TO_I2C_REG(x)	((x) >> 1)

/* ADXL355_POWER_CTL */
#define ADXL355_POWER_CTL_INSTANT_ON_TH_MSK	BIT(5)
#define ADXL355_POWER_CTL_INSTANT_ON_TH_MODE(x)	(((x) & 0x1) << 5)
#define ADXL355_POWER_CTL_FIL_SETTLE_MSK	BIT(4)
#define ADXL355_POWER_CTL_FIL_SETTLE_MODE(x)	(((x) & 0x1) << 4)
#define ADXL355_POWER_CTL_LPF_DIS_MSK		BIT(3)
#define ADXL355_POWER_CTL_LPF_DIS_MODE(x)	(((x) & 0x1) << 3)
#define ADXL355_POWER_CTL_HPF_DIS_MSK		BIT(2)
#define ADXL355_POWER_CTL_HPF_DIS_MODE(x)	(((x) & 0x1) << 2)
#define ADXL355_POWER_CTL_MODE_MSK		GENMASK(1, 0)
#define ADXL355_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL355_MEASURE */
#define ADXL355_MEASURE_AUTOSLEEP_MSK		BIT(6)
#define ADXL355_MEASURE_AUTOSLEEP_MODE(x)	(((x) & 0x1) << 6)
#define ADXL355_MEASURE_LINKLOOP_MSK		GENMASK(5, 4)
#define ADXL355_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL355_MEASURE_LOW_NOISE_MSK		BIT(3)
#define ADXL355_MEASURE_LOW_NOISE_MODE(x)	(((x) & 0x1) << 3)
#define ADXL355_MEASURE_BANDWIDTH_MSK		GENMASK(2, 0)
#define ADXL355_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL355_TIMING */
#define ADXL355_TIMING_ODR_MSK			GENMASK(7, 5)
#define ADXL355_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)
#define ADXL355_TIMING_WAKE_UP_RATE_MSK		GENMASK(4, 2)
#define ADXL355_TIMING_WAKE_UP_RATE_MODE(x)	(((x) & 0x7) << 2)
#define ADXL355_TIMING_EXT_CLK_MSK		BIT(1)
#define ADXL355_TIMING_EXT_CLK_MODE(x)		(((x) & 0x1) << 1)
#define ADXL355_TIMING_EXT_SYNC_MSK		BIT(0)
#define ADXL355_TIMING_EXT_SYNC_MODE(x)		(((x) & 0x1) << 0)

/* ADXL355_FIFO_CTL */
#define ADXL355_FIFO_CTL_FORMAT_MSK		GENMASK(5, 3)
#define ADXL355_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL355_FIFO_CTL_MODE_MSK		GENMASK(2, 1)
#define ADXL355_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL355_FIFO_CTL_SAMPLES_MSK		BIT(0)
#define ADXL355_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL355_STATUS */
#define ADXL355_STATUS_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL355_STATUS_FIFO_FULL(x)		(((x) >> 1) & 0x1)
#define ADXL355_STATUS_FIFO_OVR(x)		(((x) >> 2) & 0x1)
#define ADXL355_STATUS_ACTIVITY(x)		(((x) >> 3) & 0x1)
#define ADXL355_STATUS_USR_NVM_BUSY(x)	(((x) >> 4) & 0x1)

/* ADXL355_INT1_MAP */
#define ADXL355_INT1_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL355_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL355_INT1_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL355_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL355_INT1_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL355_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL355_INT1_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL355_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL355_INT1_MAP_INACT_MSK		BIT(4)
#define ADXL355_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL355_INT1_MAP_ACT_MSK		BIT(5)
#define ADXL355_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL355_INT1_MAP_AWAKE_MSK		BIT(6)
#define ADXL355_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL355_INT1_MAP_LOW_MSK		BIT(7)
#define ADXL355_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL355_INT2_MAP */
#define ADXL355_INT2_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL355_INT2_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL355_INT2_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL355_INT2_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL355_INT2_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL355_INT2_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL355_INT2_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL355_INT2_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL355_INT2_MAP_INACT_MSK		BIT(4)
#define ADXL355_INT2_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL355_INT2_MAP_ACT_MSK		BIT(5)
#define ADXL355_INT2_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL355_INT2_MAP_AWAKE_MSK		BIT(6)
#define ADXL355_INT2_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL355_INT2_MAP_LOW_MSK		BIT(7)
#define ADXL355_INT2_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL355_HPF */
#define ADXL355_HPF_CORNER(x)			(((x) & 0x3) << 0)

enum adxl355_axis {
	ADXL355_X_AXIS,
	ADXL355_Y_AXIS,
	ADXL355_Z_AXIS
};

enum adxl355_op_mode {
	ADXL355_STANDBY,
	ADXL355_WAKE_UP,
	ADXL355_INSTANT_ON,
	ADXL355_FULL_BW_MEASUREMENT
};

enum adxl355_bandwidth {
	ADXL355_BW_200HZ,
	ADXL355_BW_400HZ,
	ADXL355_BW_800HZ,
	ADXL355_BW_1600HZ,
	ADXL355_BW_3200HZ,
	ADXL355_BW_LPF_DISABLED = 0xC,
};

enum adxl355_hpf_corner {
	ADXL355_HPF_CORNER_0,
	ADXL355_HPF_CORNER_1,
	ADXL355_HPF_CORNER_2,
	ADXL355_HPF_CORNER_3,
	ADXL355_HPF_DISABLED,
};

enum adxl355_act_proc_mode {
	ADXL355_DEFAULT,
	ADXL355_LINKED,
	ADXL355_LOOPED
};

enum adxl355_odr {
	ADXL355_ODR_4000HZ,
	ADXL355_ODR_2000HZ,
	ADXL355_ODR_1000HZ,
	ADXL355_ODR_500HZ,
	ADXL355_ODR_250HZ,
	ADXL355_ODR_125HZ,
	ADXL355_ODR_62_5HZ,
	ADXL355_ODR_31_25HZ,
	ADXL355_ODR_15_625HZ,
	ADXL355_ODR_7_813HZ,
	ADXL355_ODR_3_906HZ
};

enum adxl355_instant_on_th_mode {
	ADXL355_INSTANT_ON_LOW_TH,
	ADXL355_INSTANT_ON_HIGH_TH
};

enum adxl355_wakeup_rate {
	ADXL355_WUR_52ms,
	ADXL355_WUR_104ms,
	ADXL355_WUR_208ms,
	ADXL355_WUR_512ms,
	ADXL355_WUR_2048ms,
	ADXL355_WUR_4096ms,
	ADXL355_WUR_8192ms,
	ADXL355_WUR_24576ms
};

enum adxl355_filter_settle {
	ADXL355_FILTER_SETTLE_370,
	ADXL355_FILTER_SETTLE_16
};

enum adxl355_fifo_format {
	ADXL355_XYZ_FIFO,
	ADXL355_X_FIFO,
	ADXL355_Y_FIFO,
	ADXL355_XY_FIFO,
	ADXL355_Z_FIFO,
	ADXL355_XZ_FIFO,
	ADXL355_YZ_FIFO,
	ADXL355_XYZ_PEAK_FIFO,
};

enum adxl355_fifo_mode {
	ADXL355_FIFO_BYPASSED,
	ADXL355_FIFO_STREAMED,
	ADXL355_FIFO_TRIGGERED,
	ADXL355_FIFO_OLD_SAVED
};

struct adxl355_fifo_config {
	enum adxl355_fifo_mode fifo_mode;
	enum adxl355_fifo_format fifo_format;
	u16_t fifo_samples;
};

struct adxl355_activity_threshold {
	u16_t thresh;
	bool referenced;
	bool enable;
};

struct adxl355_xyz_accel_data {
	s16_t x;
	s16_t y;
	s16_t z;
};

struct adxl355_data {
	struct device *bus;
#ifdef CONFIG_ADXL355_SPI
	struct spi_config spi_cfg;
#if defined(DT_INST_0_ADI_ADXL355_CS_GPIOS_CONTROLLER)
	struct spi_cs_control adxl355_cs_ctrl;
#endif
#endif
	struct adxl355_xyz_accel_data sample;
	struct adxl355_fifo_config fifo_config;

#ifdef CONFIG_ADXL355_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;
	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;

#if defined(CONFIG_ADXL355_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_ADXL355_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_ADXL355_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif
#endif /* CONFIG_ADXL355_TRIGGER */
};

struct adxl355_dev_config {
#ifdef CONFIG_ADXL355_I2C
	const char *i2c_port;
	u16_t i2c_addr;
#endif
#ifdef CONFIG_ADXL355_SPI
	const char *spi_port;
	u16_t spi_slave;
	u32_t spi_max_frequency;
#if defined(DT_INST_0_ADI_ADXL355_CS_GPIOS_CONTROLLER)
	const char *gpio_cs_port;
	gpio_pin_t cs_gpio;
#endif
#endif /* CONFIG_ADXL355_SPI */
#ifdef CONFIG_ADXL355_TRIGGER
	const char *gpio_port;
	gpio_pin_t int_gpio;
	gpio_dt_flags_t int_flags;
#endif
	bool max_peak_detect_mode;

	/* Device Settings */
	bool autosleep;

	struct adxl355_activity_threshold activity_th;
	struct adxl355_fifo_config fifo_config;

	enum adxl355_bandwidth bw;
	enum adxl355_hpf_corner hpf;
	enum adxl355_odr odr;
	enum adxl355_wakeup_rate wur;
	enum adxl355_act_proc_mode act_proc_mode;
	enum adxl355_instant_on_th_mode	th_mode;
	enum adxl355_filter_settle filter_settle;
	enum adxl355_op_mode op_mode;

	u8_t activity_time;
	u8_t int1_config;
	u8_t int2_config;
};

#ifdef CONFIG_ADXL355_TRIGGER
int adxl355_get_status(struct device *dev,
		       u8_t *status1, u8_t *status2, u16_t *fifo_entries);

int adxl355_reg_write_mask(struct device *dev,
			   u8_t reg_addr, u32_t mask, u8_t data);

int adxl355_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int adxl355_init_interrupt(struct device *dev);
#endif /* CONFIG_ADXL355_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ADXL355_ADXL355_H_ */
