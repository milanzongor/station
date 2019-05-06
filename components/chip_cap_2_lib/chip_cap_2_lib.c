#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "chip_cap_2_lib.h"

#define SENSOR_CHIP_CAP_ADDR 0x28   /*!< slave address for chipchap sensor */

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

float count_temperature(uint8_t byte_1, uint8_t byte_2) {
   return (float) (((byte_1 * 64 + (byte_2 >> 2) / 4) / pow(2, 14)) * 165 - 40);
}

float count_humidity(uint8_t byte_1, uint8_t byte_2) {
   return (float) ((((byte_1 & 0x3F) * 256 + byte_2) / pow(2, 14)) * 100);
}

esp_err_t i2c_master_init(i2c_mode_t i2c_master_num, uint32_t i2c_master_freq_hz, gpio_num_t i2c_master_sda_io, gpio_num_t i2c_master_scl_io)
{
   int i2c_master_port = i2c_master_num;
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = i2c_master_sda_io;
   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
   conf.scl_io_num = i2c_master_scl_io;
   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
   conf.master.clk_speed = i2c_master_freq_hz;
   i2c_param_config(i2c_master_port, &conf);
   return i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t chip_cap_read_hum_temp(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
   int ret;
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, SENSOR_CHIP_CAP_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
   if (size > 1) {
       i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}