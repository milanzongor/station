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
include u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/depend.make

# Include the progress variables for this target.
include u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/progress.make

# Include the compile flags for this target's objects.
include u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/flags.make

u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj: u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/flags.make
u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj: ../components/u8g2_esp32_hal/u8g2_esp32_hal.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj   -c /home/milan/Desktop/BP/station/components/u8g2_esp32_hal/u8g2_esp32_hal.c

u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/Desktop/BP/station/components/u8g2_esp32_hal/u8g2_esp32_hal.c > CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.i

u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/Desktop/BP/station/components/u8g2_esp32_hal/u8g2_esp32_hal.c -o CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.s

# Object files for target u8g2_esp32_hal
u8g2_esp32_hal_OBJECTS = \
"CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj"

# External object files for target u8g2_esp32_hal
u8g2_esp32_hal_EXTERNAL_OBJECTS =

u8g2_esp32_hal/libu8g2_esp32_hal.a: u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/u8g2_esp32_hal.c.obj
u8g2_esp32_hal/libu8g2_esp32_hal.a: u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/build.make
u8g2_esp32_hal/libu8g2_esp32_hal.a: u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libu8g2_esp32_hal.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && $(CMAKE_COMMAND) -P CMakeFiles/u8g2_esp32_hal.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/u8g2_esp32_hal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/build: u8g2_esp32_hal/libu8g2_esp32_hal.a

.PHONY : u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/build

u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal && $(CMAKE_COMMAND) -P CMakeFiles/u8g2_esp32_hal.dir/cmake_clean.cmake
.PHONY : u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/clean

u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/Desktop/BP/station/components/u8g2_esp32_hal /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal /home/milan/Desktop/BP/station/cmake-build-debug/u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : u8g2_esp32_hal/CMakeFiles/u8g2_esp32_hal.dir/depend

