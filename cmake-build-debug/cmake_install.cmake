# Install script for directory: /home/milan/Desktop/BP/station

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/milan/Desktop/BP/station/cmake-build-debug/soc/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/log/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/heap/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/xtensa-debug-module/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/app_trace/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/freertos/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/vfs/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/newlib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_ringbuf/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/driver/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_event/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/ethernet/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/lwip/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/tcpip_adapter/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/partition_table/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/app_update/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/spi_flash/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/mbedtls/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/micro-ecc/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/bootloader_support/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/nvs_flash/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/pthread/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/smartconfig_ack/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/wpa_supplicant/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp32/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/cxx/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/sdmmc/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/chip_cap_2_lib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/max31865_lib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/mics6814_lib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/mpl115a2_lib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/scd30_lib/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/asio/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/jsmn/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/aws_iot/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/bootloader/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/bt/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/coap/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/console/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/nghttp/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp-tls/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_adc_cal/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/tcp_transport/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_http_client/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_http_server/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/esptool_py/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/expat/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/wear_levelling/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/fatfs/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/freemodbus/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/idf_test/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/json/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/libsodium/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/mdns/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/mqtt/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/openssl/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/protobuf-c/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/protocomm/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/spiffs/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/u8g2/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/ulp/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/unity/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/wifi_provisioning/cmake_install.cmake")
  include("/home/milan/Desktop/BP/station/cmake-build-debug/main/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/milan/Desktop/BP/station/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
