# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/milan/.app/clion-2018.2.6/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/milan/.app/clion-2018.2.6/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/milan/Desktop/BP/station

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/milan/Desktop/BP/station/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/station.elf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/station.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/station.elf.dir/flags.make

dummy_main_src.c:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dummy_main_src.c"
	/home/milan/.app/clion-2018.2.6/bin/cmake/linux/bin/cmake -E touch dummy_main_src.c

CMakeFiles/station.elf.dir/dummy_main_src.c.obj: CMakeFiles/station.elf.dir/flags.make
CMakeFiles/station.elf.dir/dummy_main_src.c.obj: dummy_main_src.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/station.elf.dir/dummy_main_src.c.obj"
	/home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/station.elf.dir/dummy_main_src.c.obj   -c /home/milan/Desktop/BP/station/cmake-build-debug/dummy_main_src.c

CMakeFiles/station.elf.dir/dummy_main_src.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/station.elf.dir/dummy_main_src.c.i"
	/home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/Desktop/BP/station/cmake-build-debug/dummy_main_src.c > CMakeFiles/station.elf.dir/dummy_main_src.c.i

CMakeFiles/station.elf.dir/dummy_main_src.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/station.elf.dir/dummy_main_src.c.s"
	/home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/Desktop/BP/station/cmake-build-debug/dummy_main_src.c -o CMakeFiles/station.elf.dir/dummy_main_src.c.s

# Object files for target station.elf
station_elf_OBJECTS = \
"CMakeFiles/station.elf.dir/dummy_main_src.c.obj"

# External object files for target station.elf
station_elf_EXTERNAL_OBJECTS =

station.elf: CMakeFiles/station.elf.dir/dummy_main_src.c.obj
station.elf: CMakeFiles/station.elf.dir/build.make
station.elf: soc/libsoc.a
station.elf: log/liblog.a
station.elf: heap/libheap.a
station.elf: xtensa-debug-module/libxtensa-debug-module.a
station.elf: app_trace/libapp_trace.a
station.elf: freertos/libfreertos.a
station.elf: vfs/libvfs.a
station.elf: newlib/libnewlib.a
station.elf: esp_ringbuf/libesp_ringbuf.a
station.elf: driver/libdriver.a
station.elf: esp_event/libesp_event.a
station.elf: ethernet/libethernet.a
station.elf: lwip/liblwip.a
station.elf: tcpip_adapter/libtcpip_adapter.a
station.elf: app_update/libapp_update.a
station.elf: spi_flash/libspi_flash.a
station.elf: mbedtls/libmbedtls.a
station.elf: micro-ecc/libmicro-ecc.a
station.elf: bootloader_support/libbootloader_support.a
station.elf: nvs_flash/libnvs_flash.a
station.elf: pthread/libpthread.a
station.elf: smartconfig_ack/libsmartconfig_ack.a
station.elf: wpa_supplicant/libwpa_supplicant.a
station.elf: esp32/libesp32.a
station.elf: cxx/libcxx.a
station.elf: sdmmc/libsdmmc.a
station.elf: chip_cap_2_lib/libchip_cap_2_lib.a
station.elf: max31865_lib/libmax31865_lib.a
station.elf: mics6814_lib/libmics6814_lib.a
station.elf: mpl115a2_lib/libmpl115a2_lib.a
station.elf: scd30_lib/libscd30_lib.a
station.elf: u8g2_esp32_hal/libu8g2_esp32_hal.a
station.elf: asio/libasio.a
station.elf: jsmn/libjsmn.a
station.elf: coap/libcoap.a
station.elf: console/libconsole.a
station.elf: nghttp/libnghttp.a
station.elf: esp-tls/libesp-tls.a
station.elf: esp_adc_cal/libesp_adc_cal.a
station.elf: tcp_transport/libtcp_transport.a
station.elf: esp_http_client/libesp_http_client.a
station.elf: esp_http_server/libesp_http_server.a
station.elf: esp_https_ota/libesp_https_ota.a
station.elf: expat/libexpat.a
station.elf: wear_levelling/libwear_levelling.a
station.elf: fatfs/libfatfs.a
station.elf: freemodbus/libfreemodbus.a
station.elf: json/libjson.a
station.elf: libsodium/liblibsodium.a
station.elf: mdns/libmdns.a
station.elf: mqtt/libmqtt.a
station.elf: openssl/libopenssl.a
station.elf: protobuf-c/libprotobuf-c.a
station.elf: protocomm/libprotocomm.a
station.elf: spiffs/libspiffs.a
station.elf: ulp/libulp.a
station.elf: unity/libunity.a
station.elf: wifi_provisioning/libwifi_provisioning.a
station.elf: main/libmain.a
station.elf: /home/milan/esp/esp-idf/components/esp32/libhal.a
station.elf: esp32/esp32.common.ld
station.elf: esp32/esp32_out.ld
station.elf: /home/milan/esp/esp-idf/components/esp32/ld/esp32.rom.ld
station.elf: /home/milan/esp/esp-idf/components/esp32/ld/esp32.peripherals.ld
station.elf: /home/milan/esp/esp-idf/components/esp32/ld/esp32.rom.libgcc.ld
station.elf: /home/milan/esp/esp-idf/components/esp32/ld/esp32.rom.spiram_incompatible_fns.ld
station.elf: CMakeFiles/station.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable station.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/station.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/station.elf.dir/build: station.elf

.PHONY : CMakeFiles/station.elf.dir/build

CMakeFiles/station.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/station.elf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/station.elf.dir/clean

CMakeFiles/station.elf.dir/depend: dummy_main_src.c
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/Desktop/BP/station /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles/station.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/station.elf.dir/depend

