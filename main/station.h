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

