#ifndef _CHIP_CAP_2_LIB_H_
#define _CHIP_CAP_2_LIB_H_

#include "esp_err.h"
#include "driver/i2c.h"

float count_temperature(uint8_t byte_1, uint8_t byte_2);
float count_humidity(uint8_t byte_1, uint8_t byte_2);
esp_err_t i2c_master_init(i2c_mode_t i2c_master_num, uint32_t i2c_master_freq_hz, gpio_num_t i2c_master_sda_io, gpio_num_t i2c_master_scl_io);
esp_err_t chip_cap_read_hum_temp(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);

#endif


