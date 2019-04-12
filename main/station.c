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
#define SENSOR_CHIP_CAP_ADDR 0x28   /*!< slave address for chipchap sensor */

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

static const char *TAG = "Station_tag";

void float_to_str(float float_var, char *out_buf){
    enum {BufSize=9};
    char buf[BufSize];
    snprintf (buf, BufSize, "%2.2f", float_var);
    memcpy(out_buf, buf, BufSize*sizeof(*out_buf));
}

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


esp_err_t chip_cap_read_hum_temp(i2c_port_t i2c_num, uint8_t *hum_1, uint8_t *hum_2, uint8_t *temp_1, uint8_t *temp_2)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_CHIP_CAP_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, hum_1, ACK_VAL);
    i2c_master_read_byte(cmd, hum_2, ACK_VAL);
    i2c_master_read_byte(cmd, temp_1, ACK_VAL);
    i2c_master_read_byte(cmd, temp_2, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
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
    i2c_master_write_byte(cmd, SENSOR_CHIP_CAP_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
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
    i2c_master_write_byte(cmd, SENSOR_CHIP_CAP_ADDR << 1 | I2C_MASTER_WRITE, 1 /* expect ack */); // header C2
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


int count_co2(uint8_t *data_rd) {
    unsigned int co2;
    co2 = (unsigned int)((((unsigned int)data_rd[0]) << 24) |
                             (((unsigned int)data_rd[1]) << 16) |
                             (((unsigned int)data_rd[3]) << 8) |
                             ((unsigned int)data_rd[4]));
    return (int)(*(float*)&co2);
}


//static void i2c_co2_task(void *arg)
//{
//    int ret;
//    int cnt = 0;
//    size_t co2_data_length = 6;
//    uint8_t *co2_data_arr = (uint8_t *)malloc(co2_data_length);
//    uint32_t task_idx = (uint32_t)arg;
//    int co2;
//
//    ESP_ERROR_CHECK(SCD30_set_measurement_interval(I2C_MASTER_NUM)); // set measurement interval to 2s
//    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init
//    ESP_ERROR_CHECK(SCD30_start_periodic_measurement(I2C_MASTER_NUM)); // start periodic measurements
//    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init
//
//
//    while (1) {
//        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
//        ret = SCD30_read_measurement_buffer(I2C_MASTER_NUM, co2_data_arr, co2_data_length);
//
//        if (ret == ESP_ERR_TIMEOUT) {
//            ESP_LOGE(TAG, "I2C Timeout");
//        } else if (ret == ESP_OK) {
//            co2 = count_co2(co2_data_arr);
//            printf("INFO[%d]  CO2 value is: %d \n", task_idx, co2);
//
//            if(xQueueSendToBack(demo_queue,&co2,1000/portTICK_RATE_MS)!=pdTRUE) {
//                printf("WARNING  Fail to queue value %d", co2);
//            }
//
//        } else {
//            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
//        }
//        vTaskDelay(MEASUREMENT_DELAY / portTICK_RATE_MS);
//    }
//    vTaskDelete(NULL);
//}


static void display_task(void *arg)
{
    int co2 = 555;

    int ret;
    float temperature, humidity;
    uint8_t humidity_1, humidity_2, temperature_1, temperature_2;

    enum {BufSize=9};
    char buf[BufSize];

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

    while(1) {
        ret = chip_cap_read_hum_temp(I2C_MASTER_NUM, &humidity_1, &humidity_2, &temperature_1, &temperature_2);
        vTaskDelay(100 / portTICK_RATE_MS);

        if (ret == ESP_OK) {
            temperature = count_temperature(temperature_1, temperature_2);
            humidity = count_humidity(humidity_1, humidity_2);
            printf("CHIP_CAP ---> Humidity: %2.2f, Temperature:  %2.2f \n", humidity, temperature);


            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_timB10_tr);
            u8g2_DrawStr(&u8g2, 0,15,"ChipCap: H:");
            float_to_str(humidity, buf);
            u8g2_DrawStr(&u8g2, 80, 15, buf);
            u8g2_DrawStr(&u8g2, 120,15,"T:");
            float_to_str(temperature, buf);
            u8g2_DrawStr(&u8g2, 140, 15, buf);

            u8g2_SendBuffer(&u8g2);
        }
    }

    vTaskDelete(NULL);
}



void app_main()
{
    printf("ESP32 project @zongomil \n");
    ESP_ERROR_CHECK(i2c_master_init(I2C_MASTER_NUM, I2C_MASTER_FREQ_HZ, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    xTaskCreate(display_task /* Pointer to task */,
                "display_task" /* Name of task */,
                1024 * 100 /* Stack depth in bytes */,
                (void *)0, /* Pointer as parameter for the task */
                10, /* Priority of task */
                NULL /* Handle of created task */);

    printf("End of main loop \n");

}



