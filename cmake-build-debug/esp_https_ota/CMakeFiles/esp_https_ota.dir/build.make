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
include esp_https_ota/CMakeFiles/esp_https_ota.dir/depend.make

# Include the progress variables for this target.
include esp_https_ota/CMakeFiles/esp_https_ota.dir/progress.make

# Include the compile flags for this target's objects.
include esp_https_ota/CMakeFiles/esp_https_ota.dir/flags.make

esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj: esp_https_ota/CMakeFiles/esp_https_ota.dir/flags.make
esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj: /home/milan/esp/esp-idf/components/esp_https_ota/src/esp_https_ota.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj   -c /home/milan/esp/esp-idf/components/esp_https_ota/src/esp_https_ota.c

esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/esp_https_ota/src/esp_https_ota.c > CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.i

esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/esp_https_ota/src/esp_https_ota.c -o CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.s

# Object files for target esp_https_ota
esp_https_ota_OBJECTS = \
"CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj"

# External object files for target esp_https_ota
esp_https_ota_EXTERNAL_OBJECTS =

esp_https_ota/libesp_https_ota.a: esp_https_ota/CMakeFiles/esp_https_ota.dir/src/esp_https_ota.c.obj
esp_https_ota/libesp_https_ota.a: esp_https_ota/CMakeFiles/esp_https_ota.dir/build.make
esp_https_ota/libesp_https_ota.a: esp_https_ota/CMakeFiles/esp_https_ota.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libesp_https_ota.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && $(CMAKE_COMMAND) -P CMakeFiles/esp_https_ota.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/esp_https_ota.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp_https_ota/CMakeFiles/esp_https_ota.dir/build: esp_https_ota/libesp_https_ota.a

.PHONY : esp_https_ota/CMakeFiles/esp_https_ota.dir/build

esp_https_ota/CMakeFiles/esp_https_ota.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota && $(CMAKE_COMMAND) -P CMakeFiles/esp_https_ota.dir/cmake_clean.cmake
.PHONY : esp_https_ota/CMakeFiles/esp_https_ota.dir/clean

esp_https_ota/CMakeFiles/esp_https_ota.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/esp_https_ota /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota /home/milan/Desktop/BP/station/cmake-build-debug/esp_https_ota/CMakeFiles/esp_https_ota.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp_https_ota/CMakeFiles/esp_https_ota.dir/depend

