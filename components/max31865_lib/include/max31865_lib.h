#ifndef _MPL115A2_LIB_H_
#define _MPL115A2_LIB_H_

#include "esp_err.h"
#include "driver/i2c.h"


//---PRESSURE-SENSOR----------------------------------------------------------------------------------------------------
esp_err_t sensor_pressure_read_values(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
esp_err_t sensor_pressure_read_coefficients(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
void sensor_pressure_count_coefficients(float *A0, float *B1, float *B2, float *C12, uint8_t *pressure_coeffs_arr);
float sensor_pressure_count_temp(uint8_t *pressure_arr);
float sensor_pressure_count_pres(uint8_t *pressure_arr, float A0, float B1, float B2, float C12);

#endif


