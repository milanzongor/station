#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

typedef struct {
	char name[20];
	int co2_value;
	float temperature;
	float humidity;
} sensor_scd30_struct;

typedef struct {
	char name[20];
	float pressure;
	float temperature;
} sensor_mpl115a2_struct;

typedef struct {
	char name[20];
	float humidity;
	float temperature;
} sensor_chip_cap_struct;

typedef struct {
	char name[20];
	float temperature;
} sensor_max31865_struct;

typedef struct {
	sensor_scd30_struct scd30;
	sensor_mpl115a2_struct mpl115a2;
	sensor_chip_cap_struct chip_cap;
	sensor_max31865_struct max31865;
} sensor_data_struct;

sensor_data_struct data;

