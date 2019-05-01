#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <coap/str.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "/home/milan/Desktop/BP/station/libraries/u8g2_esp32_hal.h"
#include "/home/milan/esp/esp-idf/components/u8g2/csrc/u8g2.h"
#include "station.h"

// for I2C
#define I2C_MASTER_SCL_IO 26               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define SCD30_RDY_PIN_NUM 39
#define SENSOR_CO2_ADDR 0x61   /*!< slave address for chipchap sensor */
#define SENSOR_CHIP_CAP_ADDR 0x28   /*!< slave address for chipchap sensor */
#define SENSOR_PRESSURE_ADDR 0x60   /*!< slave address for chipchap sensor */


// display VSPI
#define PIN_CLK 18
#define PIN_MOSI 23
#define PIN_RESET -1
#define PIN_DC 21
#define PIN_CS 5

// max31865 sensor HSPI
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15
#define PIN_NUM_DRDY 36
#define PIN_NUM_DC   -1
#define PIN_NUM_RST  -1

#define PIN_NUM_LED 22
#define PIN_NUM_MS_IN 34
#define PIN_NUM_MS_OUT 27

#define WIFI_SSID "esp_wifi"
#define WIFI_PASS "mypassword"
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;


static uint8_t max31865_read[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 9 bytes to read
static size_t max31865_read_length = sizeof(max31865_read); // equals 9
static double Rref = 360.19; // reference resistor for PT100

static const char *TAG = "Station_tag";

const static char http_html_hdr[] =
        "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";


const static char page_01[] = "<!DOCTYPE html>\n"
                              "<html>\n"
                              "  <head>\n"
                              "    <title>ESP32 Weather Station</title>\n"
                              "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta http-equiv=\"refresh\" content=\"2\" />\n"
                              "    <link rel=\"icon\" href=\"data:,\">\n"
                              "    <script>\n"
                              "      function DisplayCurrentTime() {\n"
                              "          var date = new Date();\n"
                              "          var hours = date.getHours() < 10 ? \"0\" + date.getHours() : date.getHours();\n"
                              "          var minutes = date.getMinutes() < 10 ? \"0\" + date.getMinutes() : date.getMinutes();\n"
                              "          var seconds = date.getSeconds() < 10 ? \"0\" + date.getSeconds() : date.getSeconds();\n"
                              "          time = hours + \":\" + minutes + \":\" + seconds;\n"
                              "          var currentTime = document.getElementById(\"currentTime\");\n"
                              "          currentTime.innerHTML = time;\n"
                              "      };\n"
                              "      function GetReadings() {\n"
                              "      \tnocache = \"&nocache\";\n"
                              "      \tvar request = new XMLHttpRequest();\n"
                              "      \trequest.onreadystatechange = function() {\n"
                              "    \t\t\tif (this.status == 200) {\n"
                              "    \t\t\t\tif (this.responseXML != null) {\n"
                              "    \t\t\t\t\t// XML file received - contains sensor readings\n"
                              "    \t\t\t\t\tvar count;\n"
                              "    \t\t\t\t\tvar num_an = this.responseXML.getElementsByTagName('reading').length;\n"
                              "    \t\t\t\t\tfor (count = 0; count < num_an; count++) {\n"
                              "    \t\t\t\t\t\tdocument.getElementsByClassName(\"reading\")[count].innerHTML =\n"
                              "    \t\t\t\t\t  this.responseXML.getElementsByTagName('reading')[count].childNodes[0].nodeValue;\n"
                              "    \t\t\t\t\t}\n"
                              "    \t\t\t\t}\n"
                              "    \t\t\t}\n"
                              "      \t}\n"
                              "      }\n"
                              "      document.addEventListener('DOMContentLoaded', function() {\n"
                              "        DisplayCurrentTime();\n"
                              "        GetReadings();\n"
                              "      }, false);\n"
                              "    </script>\n"
                              "    <style>\n"
                              "      body {\n"
                              "        text-align: center;\n"
                              "        font-family: \"Trebuchet MS\", Arial;\n"
                              "      }\n"
                              "      table {\n"
                              "        border-collapse: collapse;\n"
                              "        width:60%;\n"
                              "        margin-left:auto;\n"
                              "        margin-right:auto;\n"
                              "      }\n"
                              "      th {\n"
                              "        padding: 16px;\n"
                              "        background-color: #0043af;\n"
                              "        color: white;\n"
                              "      }\n"
                              "      tr {\n"
                              "        border: 1px solid #ddd;\n"
                              "        padding: 16px;\n"
                              "      }\n"
                              "      tr:hover {\n"
                              "        background-color: #bcbcbc;\n"
                              "      }\n"
                              "      td {\n"
                              "        border: none;\n"
                              "        padding: 16px;\n"
                              "      }\n"
                              "      .sensor {\n"
                              "        color:white;\n"
                              "        font-weight: bold;\n"
                              "        background-color: #bcbcbc;\n"
                              "        padding: 8px;\n"
                              "      }\n"
                              "    </style>\n"
                              "  </head>\n"
                              "  <body>\n"
                              "    <h1>ESP32 Weather Station</h1>\n"
                              "    <h3>Last update: <span id=\"currentTime\"></span></h3>\n"
                              "    <table>\n"
                              "      <tr>\n"
                              "        <th>SENSOR</th>\n"
                              "        <th>MEASUREMENT</th>\n"
                              "        <th>VALUE</th>\n"
                              "      </tr>";

const static char page_02[] = "</table>\n"
                              "  </body>\n"
                              "</html>";

void float_to_str(float float_var, char *out_buf){
    enum {BufSize=9};
    char buf[BufSize];
    snprintf (buf, BufSize, "%.2f", float_var);
    memcpy(out_buf, buf, BufSize*sizeof(*out_buf));
}

//---WIFI---------------------------------------------------------------------------------------------------------------
//    event handler for wifi task
static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
            .ap = {
                    .ssid = WIFI_SSID,
                    .password = WIFI_PASS,
                    .max_connection = 4,
            },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.ap.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void add_table_row(char f[], char name[], char measurement[], float value, char units[]){
    char sensor_value[9];
    snprintf (sensor_value, 9, "%.2f", value);
    sprintf(f,"<tr>\n"
              "        <td><span class=\"sensor\">%s</span></td>\n"
              "        <td>%s</td>\n"
              "        <td><span class=\"reading\">%s</span>%s</td>\n"
              "      </tr>", name, measurement, sensor_value, units);

}

void format_html(char *buffer) {
    char f[512];
    add_table_row(f, data.chip_cap.name, "temperature", data.chip_cap.temperature, "*C");
    sprintf(buffer,"%s%s",page_01,f);

    add_table_row(f, data.chip_cap.name, "humidity", data.chip_cap.humidity, "%");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.scd30.name, "CO2", data.scd30.co2_value, "PPM");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.scd30.name, "temperature", data.scd30.temperature, "*C");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.scd30.name, "humidity", data.scd30.humidity, "%");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.max31865.name, "Outside temperature", data.max31865.temperature, "*C");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.mpl115a2.name, "temperature", data.mpl115a2.temperature, "*C");
    sprintf(buffer,"%s%s",buffer,f);

    add_table_row(f, data.mpl115a2.name, "pressure", data.mpl115a2.pressure, "KPa");
    sprintf(buffer,"%s%s",buffer,f);

    sprintf(buffer,"%s%s",buffer,page_02);

}

static void http_server_netconn_serve(struct netconn *conn) {
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**)&buf, &buflen);

        if( buflen >= 5 && strstr(buf,"GET /") != NULL ) {
            char *str = malloc(4096);
            if( str ) {
                format_html(str);
            }
            else {
                printf("*** ERROR allocating buffer.\n");
                return;
            }
            netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
            netconn_write(conn, str, strlen(str), NETCONN_NOCOPY);
            free(str);
        }

    }
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);
    netbuf_delete(inbuf);
}

//	http server task
static void http_server(void *pvParameters) {
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}


//---CHIP-CAP-SENSOR----------------------------------------------------------------------------------------------------
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


//---SCD30-SENSOR-------------------------------------------------------------------------------------------------------
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
//    printf("INFO:  write sent: %d \n", ret);

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
//    printf("INFO:  write sent: %d \n", ret);

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

float count_temp(uint8_t *data_rd) {
    unsigned int temp;
    temp = (unsigned int)((((unsigned int)data_rd[6]) << 24) |
                         (((unsigned int)data_rd[7]) << 16) |
                         (((unsigned int)data_rd[9]) << 8) |
                         ((unsigned int)data_rd[10]));
    return (*(float*)&temp);
}

float count_hum(uint8_t *data_rd) {
    unsigned int hum;
    hum = (unsigned int)((((unsigned int)data_rd[12]) << 24) |
                          (((unsigned int)data_rd[13]) << 16) |
                          (((unsigned int)data_rd[15]) << 8) |
                          ((unsigned int)data_rd[16]));
    return (*(float*)&hum);
}

//---MAX31865-SENSOR----------------------------------------------------------------------------------------------------
static esp_err_t max31865_read_output(spi_device_handle_t spi, uint8_t *received_data) {
    esp_err_t ret;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = max31865_read_length*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = &max31865_read;               //Data
    t.flags = SPI_TRANS_USE_RXDATA;
    ret=spi_device_transmit(spi, &t);  //Transmit!
    memcpy(received_data, t.rx_data, max31865_read_length*sizeof(*received_data));
    if (received_data == NULL) {
        printf("ERROR:  Just NULL received \n");
    }

    return ret;
}


static esp_err_t max31865_init(spi_device_handle_t spi) {
    esp_err_t ret;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=2*8;                     //Command is 8 bits
    t.tx_data[0]=0x80;              // address to set initial values
    t.tx_data[1]=0xC3;              // initial setup according to datasheet
    t.flags=SPI_TRANS_USE_TXDATA;
    ret=spi_device_transmit(spi, &t);  //Transmit!
//    printf("INFO:    MAX31865 initialized\n");

    return ret;
}


float max31865_temperature(uint8_t *rx_read){
    unsigned int RTDdata;
    unsigned int ADCcode;
    double R;
    double temp = 0.0;

    RTDdata = rx_read[2] << 8 | rx_read[3]; // MSB + LSB

    if (RTDdata & 1) {
        printf(" > Sensor connection fault");

    } else {
        ADCcode = RTDdata >> 1;
        R       = (double)ADCcode * Rref / 32768;
        temp    = ((double)ADCcode / 32) - 256;

//        printf(" > RTDdata = %04x\n", RTDdata);
//        printf(" > ADCcode = %d\n",   ADCcode);
//        printf(" > R       = %f\n",   R);
//        printf(" > temp    = %f\n",   temp);
    }

    return (float) temp;
}


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
    *C12 = (float) (((pressure_coeffs_arr[6] * 256 + pressure_coeffs_arr[7]) / 4) / 4194304.0);
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


//---DISPLAY-TASK-------------------------------------------------------------------------------------------------------
static void display_task(void *arg)
{
//    enum {BufSize=9};
    char buf[9];

    //---DISPLAY---
    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_uc1608_240x128_f(&u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    spi_device_handle_t spi2;
    spi_bus_config_t buscfg2 = {
            .miso_io_num=PIN_NUM_MISO,
            .mosi_io_num=PIN_NUM_MOSI,
            .sclk_io_num=PIN_NUM_CLK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz = 0,   // 0 means that max transfer size is 4k bytes
    };
    spi_device_interface_config_t devcfg2 = {
            .clock_speed_hz=10 * 1000 * 1000,           //Clock out at 10 MHz
            .mode=1,                                //SPI mode 1
            .spics_io_num=PIN_NUM_CS,               //CS pin
            .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };
    //---ChipCap---
    size_t chip_cap_data_length = 4;
    uint8_t *chip_cap_data_arr = (uint8_t *)malloc(chip_cap_data_length);

    //---MAX31865---
    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg2, 2)); // 1 is for dma channel
    //Attach the MAX31865 to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2));
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    max31865_init(spi2);

    //---MPL115A2---
    // Read 8 bytes of pressure_coeffs_arr from address(0x04)
    size_t pressure_coeffs_arr_size = 8;
    uint8_t *pressure_coeffs_arr = (uint8_t *)malloc(pressure_coeffs_arr_size);
    ESP_ERROR_CHECK(sensor_pressure_read_coefficients(I2C_MASTER_NUM, pressure_coeffs_arr, pressure_coeffs_arr_size));

    float A0, B1, B2, C12;
    sensor_pressure_count_coefficients(&A0, &B1, &B2, &C12, pressure_coeffs_arr);

    size_t pressure_arr_size = 8;
    uint8_t *pressure_arr = (uint8_t *)malloc(pressure_coeffs_arr_size);

    //---SCD30---
    size_t co2_data_length = 18;
    uint8_t *co2_data_arr = (uint8_t *)malloc(co2_data_length);

    ESP_ERROR_CHECK(SCD30_set_measurement_interval(I2C_MASTER_NUM)); // set measurement interval to 2s
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init
    ESP_ERROR_CHECK(SCD30_start_periodic_measurement(I2C_MASTER_NUM)); // start periodic measurements
    vTaskDelay(1000 / portTICK_RATE_MS); // wait 1000ms after init

    //---motion_sensor---
    gpio_set_direction(PIN_NUM_MS_IN, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_NUM_MS_OUT, GPIO_MODE_OUTPUT);
    int backlight_level = 0;

    while(1) {
        vTaskDelay(2000 / portTICK_RATE_MS);

        backlight_level = gpio_get_level(PIN_NUM_MS_IN);
        gpio_set_level(PIN_NUM_MS_OUT, (uint32_t) backlight_level);

        ESP_ERROR_CHECK(chip_cap_read_hum_temp(I2C_MASTER_NUM, chip_cap_data_arr, chip_cap_data_length));
        data.chip_cap.temperature = count_temperature(chip_cap_data_arr[2], chip_cap_data_arr[3]);
        data.chip_cap.humidity = count_humidity(chip_cap_data_arr[0], chip_cap_data_arr[1]);
        printf("CHIP_CAP ---> Humidity: %2.2f, Temperature:  %2.2f \n", data.chip_cap.humidity, data.chip_cap.temperature);

        uint8_t received_data[9];
        ESP_ERROR_CHECK(max31865_read_output(spi2, received_data));
        data.max31865.temperature = max31865_temperature(received_data);
        printf("MAX31865 --> Temperature: %2.2f \n", data.max31865.temperature);

        ESP_ERROR_CHECK(sensor_pressure_read_values(I2C_MASTER_NUM, pressure_arr, pressure_arr_size));
        data.mpl115a2.pressure = sensor_pressure_count_pres(pressure_arr, A0, B1, B2, C12);
        data.mpl115a2.temperature = sensor_pressure_count_temp(pressure_arr);
        printf("MPL115A2 --> Pressure: %.2f kPa, Temperature: %.2f C \n", data.mpl115a2.pressure, data.mpl115a2.temperature);

        ESP_ERROR_CHECK(SCD30_read_measurement_buffer(I2C_MASTER_NUM, co2_data_arr, co2_data_length));
        data.scd30.co2_value = count_co2(co2_data_arr);
        data.scd30.temperature = count_temp(co2_data_arr);
        data.scd30.humidity = count_hum(co2_data_arr);
        printf("SCD30 --> CO2: %d PPM, Temperature: %.2f , Humidity: %.2f \n", data.scd30.co2_value, data.scd30.temperature, data.scd30.humidity);
        printf("\n");

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_timB10_tr);

        u8g2_DrawStr(&u8g2, 0,15,"ChipCap: H:");
        float_to_str(data.chip_cap.humidity, buf);
        u8g2_DrawStr(&u8g2, 80, 15, buf);
        u8g2_DrawStr(&u8g2, 120,15,"T:");
        float_to_str(data.chip_cap.temperature, buf);
        u8g2_DrawStr(&u8g2, 140, 15, buf);

        u8g2_DrawStr(&u8g2, 0,30,"MAX31865: T:");
        float_to_str(data.max31865.temperature, buf);
        u8g2_DrawStr(&u8g2, 100, 30, buf);

        u8g2_DrawStr(&u8g2, 0,45,"MPL115A2: P:");
        float_to_str(data.mpl115a2.pressure, buf);
        u8g2_DrawStr(&u8g2, 100, 45, buf);
        u8g2_DrawStr(&u8g2, 140,45,"T:");
        float_to_str(data.mpl115a2.temperature, buf);
        u8g2_DrawStr(&u8g2, 160, 45, buf);

        u8g2_DrawStr(&u8g2, 0,60,"SCD30: C02:");
        float_to_str(data.scd30.co2_value, buf);
        u8g2_DrawStr(&u8g2, 100, 60, buf);
        u8g2_DrawStr(&u8g2, 0,75,"SCD30: T:");
        float_to_str(data.scd30.temperature, buf);
        u8g2_DrawStr(&u8g2, 100, 75, buf);
        u8g2_DrawStr(&u8g2, 140,75,"H:");
        float_to_str(data.scd30.humidity, buf);
        u8g2_DrawStr(&u8g2, 160, 75, buf);


        u8g2_SendBuffer(&u8g2);
    }

    vTaskDelete(NULL);
}



void app_main()
{
    printf("ESP32 project @zongomil \n");
    gpio_set_direction(PIN_NUM_LED, GPIO_MODE_OUTPUT); // turn on led
    gpio_set_level(PIN_NUM_LED, 1);

    ESP_ERROR_CHECK(i2c_master_init(I2C_MASTER_NUM, I2C_MASTER_FREQ_HZ, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.clk   = PIN_CLK;
    u8g2_esp32_hal.mosi  = PIN_MOSI;
    u8g2_esp32_hal.cs    = PIN_CS;
    u8g2_esp32_hal.dc    = PIN_DC;
    u8g2_esp32_hal.reset = PIN_RESET;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    strcpy(data.chip_cap.name, "ChipCap");
    strcpy(data.max31865.name, "MAX31865");
    strcpy(data.scd30.name, "SCD30");
    strcpy(data.mpl115a2.name, "MPL115A2");

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initialise_wifi();


    xTaskCreate(display_task /* Pointer to task */,
                "display_task" /* Name of task */,
                1024 * 100 /* Stack depth in bytes */,
                (void *)0, /* Pointer as parameter for the task */
                10, /* Priority of task */
                NULL /* Handle of created task */);

    xTaskCreate(&http_server,
                "http server",
                8000,
                NULL,
                5,
                NULL);

    printf("End of main loop \n");

}
