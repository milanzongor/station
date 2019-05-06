#ifndef _MAX31865_LIB_H_
#define _MAX31865_LIB_H_

#include "esp_err.h"
#include "driver/spi_master.h"

//---MAX31865-SENSOR----------------------------------------------------------------------------------------------------
esp_err_t max31865_read_output(spi_device_handle_t spi, uint8_t *received_data);
esp_err_t max31865_init(spi_device_handle_t spi);
float max31865_temperature(uint8_t *rx_read);


#endif


