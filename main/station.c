#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define RDY_GPIO 39
#define I2C_MASTER_SCL_IO 26               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define SENSOR_CO2_ADDR 0x61   /*!< slave address for chipchap sensor */
#define SENSOR_HUM_TEMP_ADDR 0x28   /*!< slave address for chipchap sensor */

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static const char *TAG = "i2c-example";
SemaphoreHandle_t print_mux = NULL;


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

float count_co2(uint8_t *data_rd) {
    unsigned int co2;
    co2 = (unsigned int)((((unsigned int)data_rd[0]) << 24) |
                             (((unsigned int)data_rd[1]) << 16) |
                             (((unsigned int)data_rd[3]) << 8) |
                             ((unsigned int)data_rd[4]));
    return *(float*)&co2;
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
//    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
//    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
//    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t humidity_1, humidity_2, temperature_1, temperature_2;
    double temperature, humidity;

    int cnt = 0;

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_temp_hum_sensor(I2C_MASTER_NUM, &humidity_1, &humidity_2, &temperature_1, &temperature_2);
        xSemaphoreTake(print_mux, portMAX_DELAY);

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ HUM&TEMP SENSOR\n", task_idx);
            printf("*******************\n");
            temperature = count_temperature(temperature_1, temperature_2);
            humidity = count_humidity(humidity_1, humidity_2);
            printf("Humidity: %2.2f, Temperature:  %2.2f \n \n", humidity, temperature);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}


static void i2c_co2_task(void *arg)
{
    int ret;
    int cnt = 0;
    size_t co2_data_length = 6;
    float co2;
    uint8_t *co2_data_arr = (uint8_t *)malloc(co2_data_length);
    uint32_t task_idx = (uint32_t)arg;

    ESP_ERROR_CHECK(SCD30_set_measurement_interval(I2C_MASTER_NUM)); // set measurement interval to 2s
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init
    ESP_ERROR_CHECK(SCD30_start_periodic_measurement(I2C_MASTER_NUM)); // start periodic measurements
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init


    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        vTaskDelay(3000 / portTICK_RATE_MS);
        ret = SCD30_read_measurement_buffer(I2C_MASTER_NUM, co2_data_arr, co2_data_length);
        xSemaphoreTake(print_mux, portMAX_DELAY);

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ CO2 SENSOR\n", task_idx);
            printf("*******************\n");
            co2 = count_co2(co2_data_arr);
            printf("CO2 value is: %d \n", (int)co2);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        printf("\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}


void app_main()
{
    printf("ESP32 project @zongomil \n");
    printf("JOJOJOJO");
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());

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

    printf("End of main loop \n");

}




