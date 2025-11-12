#ifndef BME280_H_
#define BME280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bme280_defs.h"

int8_t bme280_init(struct bme280_dev *dev);
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev);
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev);
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev);
int8_t bme280_get_sensor_settings(struct bme280_dev *dev);
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev);
int8_t bme280_soft_reset(const struct bme280_dev *dev);
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data, struct bme280_data *comp_data, struct bme280_calib_data *calib_data);

#ifdef __cplusplus
}
#endif
#endif
