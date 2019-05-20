#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "mpl115a2_lib.h"

#define SENSOR_PRESSURE_ADDR 0x60

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


//---PRESSURE-SENSOR----------------------------------------------------------------------------------------------------
esp_err_t sensor_pressure_read_values(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
   int ret;

   if (size == 0) {
      return ESP_OK;
   }

   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, SENSOR_PRESSURE_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */);
   i2c_master_write_byte(cmd, 0x12, ACK_CHECK_EN); // CMD to start conversion
   i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD empty byte
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
//    printf("INFO:  write sent: %d \n", ret);

   if (ret != ESP_OK) {
      return ret;
   }
   vTaskDelay(300 / portTICK_RATE_MS);

   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, SENSOR_PRESSURE_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */);
   i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD empty byte
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
//    printf("INFO2:  write sent: %d \n", ret);

   if (ret != ESP_OK) {
      return ret;
   }

   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (SENSOR_PRESSURE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
   if (size > 1) {
      i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
//    printf("INFO:  read sent \n");

   return ret;
}


esp_err_t sensor_pressure_read_coefficients(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
   int ret;

   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, SENSOR_PRESSURE_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */);
   i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN); // CMD MSB
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
//    printf("INFO:  write sent: %d \n", ret);

   if (ret != ESP_OK) {
      return ret;
   }
   vTaskDelay(300 / portTICK_RATE_MS);

   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (SENSOR_PRESSURE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
   if (size > 1) {
      i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
//    printf("INFO:  read sent\n");

   return ret;
}


void sensor_pressure_count_coefficients(float *A0, float *B1, float *B2, float *C12, uint8_t *pressure_coeffs_arr)
{
   *A0 = (float) ((pressure_coeffs_arr[0] * 256 + pressure_coeffs_arr[1]) / 8.0);
   *B1 = (pressure_coeffs_arr[2] * 256 + pressure_coeffs_arr[3]);
   if (*B1 > 32767)
   {
      *B1 -= 65536;
   }
   *B1 = (float) (*B1 / 8192.0);
   *B2 = (pressure_coeffs_arr[4] * 256 + pressure_coeffs_arr[5]);
   if (*B2 > 32767)
   {
      *B2 -= 65536;
   }
   *B2 = (float) (*B2 / 16384.0);
   *C12 = (float) (((float)(pressure_coeffs_arr[6] * 256 + pressure_coeffs_arr[7]) / 4) / 4194304.0);
}


float sensor_pressure_count_temp(uint8_t *pressure_arr)
{
   int temp = (pressure_arr[2] * 256 + (pressure_arr[3] & 0xC0)) / 64;
   return (float) ((temp - 498) / (-5.35) + 25.0);
}


float sensor_pressure_count_pres(uint8_t *pressure_arr, float A0, float B1, float B2, float C12)
{
   int temp = (pressure_arr[2] * 256 + (pressure_arr[3] & 0xC0)) / 64;
   int pres = (pressure_arr[0] * 256 + (pressure_arr[1] & 0xC0)) / 64;
   float pres_comp = A0 + (B1 + C12 * temp) * pres + B2 * temp;
   return (float) ((65.0 / 1023.0) * pres_comp + 50.0);
}