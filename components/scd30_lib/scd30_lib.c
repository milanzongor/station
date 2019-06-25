#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "scd30_lib.h"

#define SENSOR_CO2_ADDR 0x61 // i2c address of scd30 sensor
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


//---SCD30--------------------------------------------------------------------------------------------------------------
/*
 * Function:  sensor_pressure_count_pres
 * --------------------
 *  Function to read measurement buffer.
 *
 *  i2c_num: number of i2c port
 *  data_rd: pointer to array where measurement values are stored
 *  size: size of array
 *
 *  returns: returns ESP error messages or value ESP_OK if transaction was successful.
 */
esp_err_t SCD30_read_measurement_buffer(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    int ret;

    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_CO2_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */);
    i2c_master_write_byte(cmd, 0x03, ACK_CHECK_EN); // CMD MSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD LSB
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_CO2_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
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
 * Function:  SCD30_set_measurement_interval
 * --------------------
 *  Function to set measurement interval.
 *
 *  i2c_num: number of i2c port
 *
 *  returns: returns ESP error messages or value ESP_OK if transaction was successful.
 */
esp_err_t SCD30_set_measurement_interval(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_CO2_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
    //    Measure every 2 seconds
    i2c_master_write_byte(cmd, 0x46, ACK_CHECK_EN); // CMD MSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD LSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // interval MSB
    i2c_master_write_byte(cmd, 0x0A, ACK_CHECK_EN); // interval LSB
    i2c_master_write_byte(cmd, 0x5a, ACK_CHECK_EN); // CRC

//    Measure every 2 seconds
//    i2c_master_write_byte(cmd, 0x46, ACK_CHECK_EN); // CMD MSB
//    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD LSB
//    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // interval MSB
//    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN); // interval LSB
//    i2c_master_write_byte(cmd, 0xe3, ACK_CHECK_EN); // CRC


    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


/*
 * Function:  SCD30_start_periodic_measurement
 * --------------------
 *  Function to set periodic measurement.
 *
 *  i2c_num: number of i2c port
 *
 *  returns: returns ESP error messages or value ESP_OK if transaction was successful.
 */
esp_err_t SCD30_start_periodic_measurement(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_CO2_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD MSB
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN); // CMD LSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // pressure compensation MSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // pressure compensation LSB - deactivation if 0x0000
    i2c_master_write_byte(cmd, 0x81, ACK_CHECK_EN); // CRC
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


/*
 * Function:  count_co2
 * --------------------
 *  Function to count co2 concentration value.
 *
 *  data_rd: pointer to array where measurement values are stored
 *
 *  returns: returns int value of co2 concentration.
 */
int count_co2(uint8_t *data_rd) {
    unsigned int co2;
    co2 = (unsigned int)((((unsigned int)data_rd[0]) << 24) |
                             (((unsigned int)data_rd[1]) << 16) |
                             (((unsigned int)data_rd[3]) << 8) |
                             ((unsigned int)data_rd[4]));
    return (int)(*(float*)&co2);
}


/*
 * Function:  count_temp
 * --------------------
 *  Function to count temperature value.
 *
 *  data_rd: pointer to array where measurement values are stored
 *
 *  returns: returns float value of temperature.
 */
float count_temp(uint8_t *data_rd) {
    unsigned int temp;
    temp = (unsigned int)((((unsigned int)data_rd[6]) << 24) |
                         (((unsigned int)data_rd[7]) << 16) |
                         (((unsigned int)data_rd[9]) << 8) |
                         ((unsigned int)data_rd[10]));
    return (*(float*)&temp);
}


/*
 * Function:  count_hum
 * --------------------
 *  Function to count humidity value.
 *
 *  data_rd: pointer to array where measurement values are stored
 *
 *  returns: returns float value of humidity.
 */
float count_hum(uint8_t *data_rd) {
    unsigned int hum;
    hum = (unsigned int)((((unsigned int)data_rd[12]) << 24) |
                          (((unsigned int)data_rd[13]) << 16) |
                          (((unsigned int)data_rd[15]) << 8) |
                          ((unsigned int)data_rd[16]));
    return (*(float*)&hum);
}