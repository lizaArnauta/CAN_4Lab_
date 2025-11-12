#include "bme280.h"

#define OVERSAMPLING_SETTINGS		UINT8_C(0x07)
#define FILTER_STANDBY_SETTINGS		UINT8_C(0x18)

static int8_t put_device_to_sleep(const struct bme280_dev *dev);
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
static int8_t null_ptr_check(const struct bme280_dev *dev);
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);
static int8_t get_calib_data(struct bme280_dev *dev);
static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);
static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);

#ifdef BME280_FLOAT_ENABLE

static double compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data);
static double compensate_humidity(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data);
static  double compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data);

#else

static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data);
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data);
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data);

#endif

static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);
static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);
static int8_t set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings, const struct bme280_dev *dev);
static int8_t set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings, const struct bme280_dev *dev);
static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings);
static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings);
static int8_t set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings, const struct bme280_dev *dev);
static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings);

static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings);

static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings);

static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);

int8_t bme280_init(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t try_count = 5;
	uint8_t chip_id = 0;

	rslt = null_ptr_check(dev);
	if (rslt ==  BME280_OK) {
		while (try_count) {
			rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, dev);
			if ((rslt == BME280_OK) && (chip_id == BME280_CHIP_ID)) {
				dev->chip_id = chip_id;
				rslt = bme280_soft_reset(dev);
				if (rslt == BME280_OK) {
					rslt = get_calib_data(dev);
				}
				break;
			}
			dev->delay_ms(1);
			--try_count;
		}
		if (!try_count)
			rslt = BME280_E_DEV_NOT_FOUND;
	}

	return rslt;
}

int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);
	if (rslt == BME280_OK) {
		if (dev->intf != BME280_I2C_INTF)
			reg_addr = reg_addr | 0x80;
		rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);
		if (rslt != BME280_OK)
			rslt = BME280_E_COMM_FAIL;
	}

	return rslt;
}

int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t temp_buff[20];

	if (len > 10)
		len = 10;

	uint16_t temp_len;
	uint8_t reg_addr_cnt;

	rslt = null_ptr_check(dev);
	if ((rslt ==  BME280_OK) && (reg_addr != NULL) && (reg_data != NULL)) {
		if (len != 0) {
			temp_buff[0] = reg_data[0];
			if (dev->intf != BME280_I2C_INTF) {
				for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
					reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
			}
			if (len > 1) {
				interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
				temp_len = ((len * 2) - 1);
			} else {
				temp_len = len;
			}
			rslt = dev->write(dev->dev_id, reg_addr[0], temp_buff, temp_len);
			if (rslt != BME280_OK)
				rslt = BME280_E_COMM_FAIL;
		} else {
			rslt = BME280_E_INVALID_LEN;
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}


	return rslt;
}

int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t sensor_mode;

	rslt = null_ptr_check(dev);
	if (rslt == BME280_OK) {
		rslt = bme280_get_sensor_mode(&sensor_mode, dev);
		if ((rslt == BME280_OK) && (sensor_mode != BME280_SLEEP_MODE))
			rslt = put_device_to_sleep(dev);
		if (rslt == BME280_OK) {
			if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
				rslt = set_osr_settings(desired_settings, &dev->settings, dev);
			if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
				rslt = set_filter_standby_settings(desired_settings, &dev->settings, dev);
		}
	}

	return rslt;
}

int8_t bme280_get_sensor_settings(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data[4];

	rslt = null_ptr_check(dev);
	if (rslt == BME280_OK) {
		rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
		if (rslt == BME280_OK)
			parse_device_settings(reg_data, &dev->settings);
	}

	return rslt;
}

int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t last_set_mode;

	rslt = null_ptr_check(dev);

	if (rslt == BME280_OK) {
		rslt = bme280_get_sensor_mode(&last_set_mode, dev);
		if ((rslt == BME280_OK) && (last_set_mode != BME280_SLEEP_MODE))
			rslt = put_device_to_sleep(dev);
		if (rslt == BME280_OK)
			rslt = write_power_mode(sensor_mode, dev);
	}

	return rslt;
}

int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);

	if (rslt == BME280_OK) {
		rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, sensor_mode, 1, dev);
		*sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
	}

	return rslt;
}

int8_t bme280_soft_reset(const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_RESET_ADDR;
	uint8_t soft_rst_cmd = 0xB6;

	rslt = null_ptr_check(dev);
	if (rslt == BME280_OK) {
		rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);
		dev->delay_ms(2);
	}

	return rslt;
}

int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data[BME280_P_T_H_DATA_LEN] = {0};
	struct bme280_uncomp_data uncomp_data = {0};

	rslt = null_ptr_check(dev);

	if ((rslt == BME280_OK) && (comp_data != NULL)) {
		rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN, dev);

		if (rslt == BME280_OK) {
			bme280_parse_sensor_data(reg_data, &uncomp_data);
			rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

	data_lsb = (uint32_t)reg_data[6] << 8;
	data_msb = (uint32_t)reg_data[7];
	uncomp_data->humidity = data_msb | data_lsb;
}

int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
				     struct bme280_data *comp_data, struct bme280_calib_data *calib_data)
{
	int8_t rslt = BME280_OK;

	if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
		comp_data->temperature = 0;
		comp_data->pressure = 0;
		comp_data->humidity = 0;
		if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) {
			comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
		}
		if (sensor_comp & BME280_PRESS) {
			comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
		}
		if (sensor_comp & BME280_HUM) {
			comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

static int8_t set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings,
				const struct bme280_dev *dev)
{
	int8_t rslt = BME280_W_INVALID_OSR_MACRO;

	if (desired_settings & BME280_OSR_HUM_SEL)
		rslt = set_osr_humidity_settings(settings, dev);
	if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
		rslt = set_osr_press_temp_settings(desired_settings, settings, dev);

	return rslt;
}

static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

	ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;
	rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);
	if (rslt == BME280_OK) {
		reg_addr = BME280_CTRL_MEAS_ADDR;
		rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);
		if (rslt == BME280_OK)
			rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
	}

	return rslt;
}

static int8_t set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings,
						const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_OSR_PRESS_SEL)
			fill_osr_press_settings(&reg_data, settings);
		if (desired_settings & BME280_OSR_TEMP_SEL)
			fill_osr_temp_settings(&reg_data, settings);
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

static int8_t set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings,
						const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CONFIG_ADDR;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_FILTER_SEL)
			fill_filter_settings(&reg_data, settings);
		if (desired_settings & BME280_STANDBY_SEL)
			fill_standby_settings(&reg_data, settings);
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings)
{
	settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
	settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
	settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
	settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
	settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_PWR_CTRL_ADDR;
	uint8_t sensor_mode_reg_val;

	rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);
	if (rslt == BME280_OK) {
		sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);
		rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
	}

	return rslt;
}

static int8_t put_device_to_sleep(const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data[4];
	struct bme280_settings settings;

	rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
	if (rslt == BME280_OK) {
		parse_device_settings(reg_data, &settings);
		rslt = bme280_soft_reset(dev);
		if (rslt == BME280_OK)
			rslt = reload_device_settings(&settings, dev);
	}

	return rslt;
}

static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
	int8_t rslt;

	rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
	if (rslt == BME280_OK)
		rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, settings, dev);

	return rslt;
}

#ifdef BME280_FLOAT_ENABLE

static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
						struct bme280_calib_data *calib_data)
{
	double var1;
	double var2;
	double temperature;
	double temperature_min = -40;
	double temperature_max = 85;

	var1 = ((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_T1) / 1024.0;
	var1 = var1 * ((double)calib_data->dig_T2);
	var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data->dig_T3);
	calib_data->t_fine = (int32_t)(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

static double compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
						const struct bme280_calib_data *calib_data)
{
	double var1;
	double var2;
	double var3;
	double pressure;
	double pressure_min = 30000.0;
	double pressure_max = 110000.0;

	var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data->dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data->dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data->dig_P4) * 65536.0);
	var3 = ((double)calib_data->dig_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_P1);
	if (var1) {
		pressure = 1048576.0 - (double) uncomp_data->pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calib_data->dig_P9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((double)calib_data->dig_P8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((double)calib_data->dig_P7)) / 16.0;

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}

static double compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
						const struct bme280_calib_data *calib_data)
{
	double humidity;
	double humidity_min = 0.0;
	double humidity_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = ((double)calib_data->t_fine) - 76800.0;
	var2 = (((double)calib_data->dig_H4) * 64.0 + (((double)calib_data->dig_H5) / 16384.0) * var1);
	var3 = uncomp_data->humidity - var2;
	var4 = ((double)calib_data->dig_H2) / 65536.0;
	var5 = (1.0 + (((double)calib_data->dig_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)calib_data->dig_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)calib_data->dig_H1) * var6 / 524288.0);

	if (humidity > humidity_max)
		humidity = humidity_max;
	else if (humidity < humidity_min)
		humidity = humidity_min;

	return humidity;
}

#else

static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
						struct bme280_calib_data *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_T1 * 2));
	var1 = (var1 * ((int32_t)calib_data->dig_T2)) / 2048;
	var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_T3)) / 16384;
	calib_data->t_fine = var1 + var2;
	temperature = (calib_data->t_fine * 5 + 128) / 256;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}
#ifdef BME280_64BIT_ENABLE

static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
						const struct bme280_calib_data *calib_data)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int64_t var4;
	uint32_t pressure;
	uint32_t pressure_min = 3000000;
	uint32_t pressure_max = 11000000;

	var1 = ((int64_t)calib_data->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data->dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data->dig_P5) * 131072);
	var2 = var2 + (((int64_t)calib_data->dig_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)calib_data->dig_P3) / 256) + ((var1 * ((int64_t)calib_data->dig_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)calib_data->dig_P1) / 8589934592;

	if (var1 != 0) {
		var4 = 1048576 - uncomp_data->pressure;
		var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
		var1 = (((int64_t)calib_data->dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
		var2 = (((int64_t)calib_data->dig_P8) * var4) / 524288;
		var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data->dig_P7) * 16);
		pressure = (uint32_t)(((var4 / 2) * 100) / 128);

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#else

static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
						const struct bme280_calib_data *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_P6);
	var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)calib_data->dig_P4) * 65536);
	var3 = (calib_data->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)calib_data->dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) / 32768;
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t)var1);
		else
			pressure = (pressure / (uint32_t)var1) * 2;

		var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_P8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#endif

static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
						const struct bme280_calib_data *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = calib_data->t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data->humidity * 16384);
	var3 = (int32_t)(((int32_t)calib_data->dig_H4) * 1048576);
	var4 = ((int32_t)calib_data->dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)calib_data->dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)calib_data->dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 / 4096);

	if (humidity > humidity_max)
		humidity = humidity_max;

	return humidity;
}
#endif

static int8_t get_calib_data(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
	uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = {0};

	rslt = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN, dev);

	if (rslt == BME280_OK) {
		parse_temp_press_calib_data(calib_data, dev);

		reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;
		rslt = bme280_get_regs(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN, dev);
		if (rslt == BME280_OK) {
			parse_humidity_calib_data(calib_data, dev);
		}
	}

	return rslt;
}

static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
	uint8_t index;

	for (index = 1; index < len; index++) {
		temp_buff[(index * 2) - 1] = reg_addr[index];
		temp_buff[index * 2] = reg_data[index];
	}
}

static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
	struct bme280_calib_data *calib_data = &dev->calib_data;

	calib_data->dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data->dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data->dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data->dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data->dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data->dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data->dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data->dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data->dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data->dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data->dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data->dig_H1 = reg_data[25];

}

static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
	struct bme280_calib_data *calib_data = &dev->calib_data;
	int16_t dig_H4_lsb;
	int16_t dig_H4_msb;
	int16_t dig_H5_lsb;
	int16_t dig_H5_msb;

	calib_data->dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_H3 = reg_data[2];

	dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;

	dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
	calib_data->dig_H6 = (int8_t)reg_data[6];
}

static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
	uint8_t settings_changed = FALSE;

	if (sub_settings & desired_settings) {
		settings_changed = TRUE;
	} else {
		settings_changed = FALSE;
	}

	return settings_changed;
}

static int8_t null_ptr_check(const struct bme280_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		rslt = BME280_E_NULL_PTR;
	} else {
		rslt = BME280_OK;
	}

	return rslt;
}