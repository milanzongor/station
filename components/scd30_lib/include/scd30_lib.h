#ifndef _SCD30_LIB_H_
#define _SCD30_LIB_H_

#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t SCD30_read_measurement_buffer(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
esp_err_t SCD30_set_measurement_interval(i2c_port_t i2c_num);
esp_err_t SCD30_start_periodic_measurement(i2c_port_t i2c_num);
int count_co2(uint8_t *data_rd);
float count_temp(uint8_t *data_rd);
float count_hum(uint8_t *data_rd);

#endif


