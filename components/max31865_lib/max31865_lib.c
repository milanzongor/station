#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "max31865_lib.h"

uint8_t max31865_read[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 9 bytes to read
size_t max31865_read_length = sizeof(max31865_read); // equals 9
double Rref = 360.19; // reference resistor for PT100

//---MAX31865-SENSOR----------------------------------------------------------------------------------------------------
esp_err_t max31865_read_output(spi_device_handle_t spi, uint8_t *received_data) {
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


esp_err_t max31865_init(spi_device_handle_t spi) {
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
