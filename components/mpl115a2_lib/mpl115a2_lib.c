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

#define SENSOR_PRESSURE_ADDR 0x60 // i2c address of pressure sensor

#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer

#define ACK_CHECK_EN 0x1  // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0  // I2C master will not check ack from slave
#define ACK_VAL 0x0  // I2C ack value
#define NACK_VAL 0x1  // I2C nack value


//---PRESSURE-SENSOR----------------------------------------------------------------------------------------------------
/*
 * Function:  sensor_pressure_read_values
 * --------------------
 *  Function to read pressure values from MPL115A2.
 *
 *  i2c_num: number of i2c port
 *  data_rd: pointer to array where will be pressure values stored
 *  size: size of array
 *
 *  returns: returns ESP error messages or value ESP_OK if transaction was successful.
 */
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

   return ret;
}


/*
 * Function:  sensor_pressure_read_coefficients
 * --------------------
 *  Function to read pressure compensation coefficients from MPL115A2.
 *
 *  i2c_num: number of i2c port
 *  data_rd: pointer to array where will be coefficients stored
 *  size: size of array
 *
 *  returns: returns ESP error messages or value ESP_OK if transaction was successful.
 */
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

   return ret;
}


/*
 * Function:  sensor_pressure_count_coefficients
 * --------------------
 *  Function to count compensation coefficients from array read values.
 *
 *  A0: first compensation coefficient
 *  B1: second compensation coefficient
 *  B2: third compensation coefficient
 *  C12: fourth compensation coefficient
 *  data_rd: pointer to array where are coefficients stored
 *
 */
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


/*
 * Function:  sensor_pressure_count_temp
 * --------------------
 *  Function to count temperature.
 *
 *  data_rd: pointer to array where temperature values are stored
 *
 *  returns: returns float value of temperature
 */
float sensor_pressure_count_temp(uint8_t *pressure_arr)
{
   int temp = (pressure_arr[2] * 256 + (pressure_arr[3] & 0xC0)) / 64;
   return (float) ((temp - 498) / (-5.35) + 25.0);
}


/*
 * Function:  sensor_pressure_count_pres
 * --------------------
 *  Function to count pressure.
 *
 *  data_rd: pointer to array where temperature values are stored
 *  A0: first compensation coefficient
 *  B1: second compensation coefficient
 *  B2: third compensation coefficient
 *  C12: fourth compensation coefficient
 *
 *  returns: returns float value of pressure
 */
float sensor_pressure_count_pres(uint8_t *pressure_arr, float A0, float B1, float B2, float C12)
{
   int temp = (pressure_arr[2] * 256 + (pressure_arr[3] & 0xC0)) / 64;
   int pres = (pressure_arr[0] * 256 + (pressure_arr[1] & 0xC0)) / 64;
   float pres_comp = A0 + (B1 + C12 * temp) * pres + B2 * temp;
   return (float) ((65.0 / 1023.0) * pres_comp + 50.0);
}