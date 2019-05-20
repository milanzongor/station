#ifndef _MICS6814_LIB_H_
#define _MICS6814_LIB_H_

#include "esp_err.h"
#include "driver/adc.h"

#define NO_OF_SAMPLES   64          //Multisampling
#define MAX_VOLTAGE 3300.0
#define R0_CO 1200000.0               //1.2 MOhm
#define R_CO 1056000.0


float count_co_ratio(int output_voltage);
double calculate_gas(int gas, int co_sensor_voltage);
uint32_t read_adc(adc_channel_t channel);

#endif


