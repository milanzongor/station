#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "mics6814_lib.h"

enum{CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH};

float count_co_ratio(int output_voltage){
//   float Rs = (MAX_VOLTAGE/output_voltage - 1)*R_CO;
    float Rs = R_CO*output_voltage/(MAX_VOLTAGE - output_voltage);
   return Rs/R0_CO;
}

double calculate_gas(int gas, int co_sensor_voltage)
{
   double ratio0 = 0.00; //NH3
   double ratio1 = ((MAX_VOLTAGE/co_sensor_voltage- 1)*R_CO) / R0_CO; //CO sensor
   double ratio2 = 0.00; //NO2

   double c = 0;
   switch(gas)
   {
      case CO:
      {
         c = pow(ratio1, -1.179)*4.385;
         break;
      }
      case NO2:
      {
         c = pow(ratio2, 1.007)/6.855;
         break;
      }
      case NH3:
      {
         c = pow(ratio0, -1.67)/1.47;
         break;
      }
      case C3H8:
      {
         c = pow(ratio0, -2.518)*570.164;
         break;
      }
      case C4H10:
      {
         c = pow(ratio0, -2.138)*398.107;
         break;
      }
      case CH4:
      {
         c = pow(ratio1, -4.363)*630.957;
         break;
      }
      case H2:
      {
         c = pow(ratio1, -1.8)*0.73;
         break;
      }
      case C2H5OH:
      {
         c = pow(ratio1, -1.552)*1.622;
         break;
      }
      default:
         break;
   }

   return c;
}

uint32_t read_adc(adc_channel_t channel){
   uint32_t adc_reading = 0;
   //Multisampling
   for (int i = 0; i < NO_OF_SAMPLES; i++) {
      adc_reading += adc1_get_raw((adc1_channel_t)channel);
   }
   adc_reading /= NO_OF_SAMPLES;
   return adc_reading;
}


