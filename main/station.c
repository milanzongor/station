#include <stdio.h>
#include <math.h>
#include <coap/str.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "/home/milan/Desktop/BP/station/libraries/u8g2_esp32_hal.h"
#include "/home/milan/esp/esp-idf/components/u8g2/csrc/u8g2.h"


#define I2C_MASTER_SCL_IO 26               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define SENSOR_CO2_ADDR 0x61   /*!< slave address for chipchap sensor */
#define SENSOR_HUM_TEMP_ADDR 0x28   /*!< slave address for chipchap sensor */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define RDY_GPIO 39

#define PIN_CLK 18
#define PIN_MOSI 23
#define PIN_RESET -1
#define PIN_DC 21
#define PIN_CS 5

#define MEASUREMENT_DELAY 10000 /* 10s delay */

static const char *TAG = "Station_tag";
xQueueHandle demo_queue;

struct sensor_hum_temp {
    float humidity;
    float temperature;
    int co2;
};



/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


static esp_err_t SCD30_read_measurement_buffer(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
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
//    printf("INFO:  write sent: %d \n", ret);

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
//    printf("INFO:  read sent \n");

    return ret;
}


static esp_err_t SCD30_set_measurement_interval(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_HUM_TEMP_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
    i2c_master_write_byte(cmd, 0x46, ACK_CHECK_EN); // CMD MSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD LSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // interval MSB
    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN); // interval LSB
    i2c_master_write_byte(cmd, 0xe3, ACK_CHECK_EN); // CRC
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    printf("INFO:  write sent: %d \n", ret);

    return ret;
}


static esp_err_t SCD30_start_periodic_measurement(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_HUM_TEMP_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // CMD MSB
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN); // CMD LSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // preasure compensation MSB
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // preasure compensation LSB - deactivation if 0x0000
    i2c_master_write_byte(cmd, 0x81, ACK_CHECK_EN); // CRC
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    printf("INFO:  write sent: %d \n", ret);

    return ret;
}


/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_temp_hum_sensor(i2c_port_t i2c_num, uint8_t *hum_1, uint8_t *hum_2, uint8_t *temp_1, uint8_t *temp_2)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_HUM_TEMP_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, hum_1, ACK_VAL);
    i2c_master_read_byte(cmd, hum_2, ACK_VAL);
    i2c_master_read_byte(cmd, temp_1, ACK_VAL);
    i2c_master_read_byte(cmd, temp_2, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


double count_temperature(uint8_t byte_1, uint8_t byte_2){
    return ((byte_1*64 + (byte_2 >> 2)/4) / pow(2,14)) * 165 - 40;
}


double count_humidity(uint8_t byte_1, uint8_t byte_2){
    return (((byte_1 & 0x3F)*256 + byte_2) / pow(2,14)) * 100;
}


int count_co2(uint8_t *data_rd) {
    unsigned int co2;
    co2 = (unsigned int)((((unsigned int)data_rd[0]) << 24) |
                             (((unsigned int)data_rd[1]) << 16) |
                             (((unsigned int)data_rd[3]) << 8) |
                             ((unsigned int)data_rd[4]));
    return (int)(*(float*)&co2);
}


/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}


static void i2c_temp_hum_task(void *arg)
{
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t humidity_1, humidity_2, temperature_1, temperature_2;
    double temperature, humidity;
    int cnt = 0;

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_temp_hum_sensor(I2C_MASTER_NUM, &humidity_1, &humidity_2, &temperature_1, &temperature_2);

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout \n");
        } else if (ret == ESP_OK) {
            temperature = count_temperature(temperature_1, temperature_2);
            humidity = count_humidity(humidity_1, humidity_2);
            printf("INFO[%d]  Humidity: %2.2f, Temperature:  %2.2f \n",task_idx, humidity, temperature);

            if(xQueueSendToBack(demo_queue,&temperature,1000/portTICK_RATE_MS)!=pdTRUE) {
                printf("WARNING  Fail to queue value %2.2f", temperature);
            }
            if(xQueueSendToBack(demo_queue,&humidity,1000/portTICK_RATE_MS)!=pdTRUE) {
                printf("WARNING  Fail to queue value %2.2f", humidity);
            }

        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        vTaskDelay(MEASUREMENT_DELAY / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}


static void i2c_co2_task(void *arg)
{
    int ret;
    int cnt = 0;
    size_t co2_data_length = 6;
    uint8_t *co2_data_arr = (uint8_t *)malloc(co2_data_length);
    uint32_t task_idx = (uint32_t)arg;
    int co2;

    ESP_ERROR_CHECK(SCD30_set_measurement_interval(I2C_MASTER_NUM)); // set measurement interval to 2s
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init
    ESP_ERROR_CHECK(SCD30_start_periodic_measurement(I2C_MASTER_NUM)); // start periodic measurements
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init


    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = SCD30_read_measurement_buffer(I2C_MASTER_NUM, co2_data_arr, co2_data_length);

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            co2 = count_co2(co2_data_arr);
            printf("INFO[%d]  CO2 value is: %d \n", task_idx, co2);

            if(xQueueSendToBack(demo_queue,&co2,1000/portTICK_RATE_MS)!=pdTRUE) {
                printf("WARNING  Fail to queue value %d", co2);
            }

        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        vTaskDelay(MEASUREMENT_DELAY / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}


static void display_task(void *arg)
{
    int cnt = 0;
    uint32_t task_idx = (uint32_t)arg;
    int co2;
    double temperature, humidity;

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.clk   = PIN_CLK;
    u8g2_esp32_hal.mosi  = PIN_MOSI;
    u8g2_esp32_hal.cs    = PIN_CS;
    u8g2_esp32_hal.dc    = PIN_DC;
    u8g2_esp32_hal.reset = PIN_RESET;
    u8g2_esp32_hal_init(u8g2_esp32_hal);


    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_uc1608_240x128_f(
            &u8g2,
            U8G2_R0,
            u8g2_esp32_spi_byte_cb,
            u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);

        if(xQueueReceive(demo_queue,&co2,60000/portTICK_RATE_MS)!=pdTRUE) {  // max wait 60s
            printf("Fail to receive queued CO2 value \n");
        } else {
            printf("INFO  Received CO2 value %d \n",co2);
        }
//        if (uxQueueMessagesWaiting(demo_queue)==0) { // no message? take a break
//            vTaskDelay(1000 / portTICK_RATE_MS); // delay 15s
//        }

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 0,15,"Hello World!");

        enum {BufSize=9};
        char buf[BufSize];
        snprintf (buf, BufSize, "%d", co2);
        u8g2_DrawStr(&u8g2, 50, 30, buf);

//        enum {BufSize1=9};
//        char buf1[BufSize1];
//        snprintf (buf1, BufSize1, "%2.2f", temperature);
//        u8g2_DrawStr(&u8g2, 50, 60, buf1);

        u8g2_SendBuffer(&u8g2);

        printf("\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}



void app_main()
{
    printf("ESP32 project @zongomil \n");
    ESP_ERROR_CHECK(i2c_master_init());
    demo_queue = xQueueCreate(10, sizeof(int));

    xTaskCreate(i2c_temp_hum_task /* Pointer to task */,
                "i2c_temp_hum_task" /* Name of task */,
                1024 * 2 /* Stack depth in bytes */,
                (void *)0, /* Pointer as parameter for the task */
                10, /* Priority of task */
                NULL /* Handle of created task */);

    xTaskCreate(i2c_co2_task /* Pointer to task */,
                "i2c_co2_task" /* Name of task */,
                1024 * 2 /* Stack depth in bytes */,
                (void *)1, /* Pointer as parameter for the task */
                10, /* Priority of task */
                NULL /* Handle of created task */);

    xTaskCreate(display_task /* Pointer to task */,
                "display_task" /* Name of task */,
                1024 * 10 /* Stack depth in bytes */,
                (void *)2, /* Pointer as parameter for the task */
                10, /* Priority of task */
                NULL /* Handle of created task */);

    printf("End of main loop \n");

}






///* Copyright (c) 2017 pcbreflux. All Rights Reserved.
// *
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, version 3.
// *
// * This program is distributed in the hope that it will be useful, but
// * WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// * General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with this program. If not, see <http://www.gnu.org/licenses/>. *
// */
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//
//#include "sdkconfig.h"
//
//#include "esp_system.h"
//#include "esp_heap_alloc_caps.h"
//
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/queue.h"
//#include "freertos/semphr.h"
//
//SemaphoreHandle_t demo_semaphore1;
//int jojo = 888;
//
//// let the rx_task1 wait for us
//void tx_task1(void *arg) {
//    uint32_t txpos=100;
//
//    demo_semaphore1 = xSemaphoreCreateBinary();
//
//    while (1) {
//        vTaskDelay(10000 / portTICK_RATE_MS); // delay 10s
//        printf("tx_task1 notify %d \n",txpos);
//        xSemaphoreGive(demo_semaphore1);
//        jojo = jojo + 10;
//        txpos++;
//    }
//
//}
//
//// wait for tx_task1 to give semaphore
//void rx_task1(void *arg) {
//    uint32_t rxpos=0;
//
//    while (1) {
//        if (demo_semaphore1!=NULL) {
//            printf("rx_task1 semaphore yield \n");
//            if(xSemaphoreTake(demo_semaphore1,60000/portTICK_RATE_MS)!=pdTRUE) {  // max wait 60s
//                printf("rx_task1 fail to receive semaphore\n");
//            } else {
//                printf("rx_task1 get semaphore %d \n",rxpos);
//                printf("jojo %d \n", jojo);
//            }
//        } else {
//            vTaskDelay(100 / portTICK_RATE_MS); // delay 100ms
//        }
//        rxpos++;
//    }
//}
//
//
//
//void app_main() {
//
//    xTaskCreate(tx_task1, "tx_task1", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 5, NULL);
//    xTaskCreate(rx_task1, "rx_task1", CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE, NULL, 5, NULL);
//
//}
