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
include mdns/CMakeFiles/mdns.dir/depend.make

# Include the progress variables for this target.
include mdns/CMakeFiles/mdns.dir/progress.make

# Include the compile flags for this target's objects.
include mdns/CMakeFiles/mdns.dir/flags.make

mdns/CMakeFiles/mdns.dir/mdns.c.obj: mdns/CMakeFiles/mdns.dir/flags.make
mdns/CMakeFiles/mdns.dir/mdns.c.obj: /home/milan/esp/esp-idf/components/mdns/mdns.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object mdns/CMakeFiles/mdns.dir/mdns.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mdns.dir/mdns.c.obj   -c /home/milan/esp/esp-idf/components/mdns/mdns.c

mdns/CMakeFiles/mdns.dir/mdns.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mdns.dir/mdns.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/mdns/mdns.c > CMakeFiles/mdns.dir/mdns.c.i

mdns/CMakeFiles/mdns.dir/mdns.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mdns.dir/mdns.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/mdns/mdns.c -o CMakeFiles/mdns.dir/mdns.c.s

mdns/CMakeFiles/mdns.dir/mdns_console.c.obj: mdns/CMakeFiles/mdns.dir/flags.make
mdns/CMakeFiles/mdns.dir/mdns_console.c.obj: /home/milan/esp/esp-idf/components/mdns/mdns_console.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object mdns/CMakeFiles/mdns.dir/mdns_console.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mdns.dir/mdns_console.c.obj   -c /home/milan/esp/esp-idf/components/mdns/mdns_console.c

mdns/CMakeFiles/mdns.dir/mdns_console.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mdns.dir/mdns_console.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/mdns/mdns_console.c > CMakeFiles/mdns.dir/mdns_console.c.i

mdns/CMakeFiles/mdns.dir/mdns_console.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mdns.dir/mdns_console.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/mdns/mdns_console.c -o CMakeFiles/mdns.dir/mdns_console.c.s

mdns/CMakeFiles/mdns.dir/mdns_networking.c.obj: mdns/CMakeFiles/mdns.dir/flags.make
mdns/CMakeFiles/mdns.dir/mdns_networking.c.obj: /home/milan/esp/esp-idf/components/mdns/mdns_networking.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object mdns/CMakeFiles/mdns.dir/mdns_networking.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mdns.dir/mdns_networking.c.obj   -c /home/milan/esp/esp-idf/components/mdns/mdns_networking.c

mdns/CMakeFiles/mdns.dir/mdns_networking.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mdns.dir/mdns_networking.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/mdns/mdns_networking.c > CMakeFiles/mdns.dir/mdns_networking.c.i

mdns/CMakeFiles/mdns.dir/mdns_networking.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mdns.dir/mdns_networking.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/mdns/mdns_networking.c -o CMakeFiles/mdns.dir/mdns_networking.c.s

# Object files for target mdns
mdns_OBJECTS = \
"CMakeFiles/mdns.dir/mdns.c.obj" \
"CMakeFiles/mdns.dir/mdns_console.c.obj" \
"CMakeFiles/mdns.dir/mdns_networking.c.obj"

# External object files for target mdns
mdns_EXTERNAL_OBJECTS =

mdns/libmdns.a: mdns/CMakeFiles/mdns.dir/mdns.c.obj
mdns/libmdns.a: mdns/CMakeFiles/mdns.dir/mdns_console.c.obj
mdns/libmdns.a: mdns/CMakeFiles/mdns.dir/mdns_networking.c.obj
mdns/libmdns.a: mdns/CMakeFiles/mdns.dir/build.make
mdns/libmdns.a: mdns/CMakeFiles/mdns.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C static library libmdns.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && $(CMAKE_COMMAND) -P CMakeFiles/mdns.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mdns.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mdns/CMakeFiles/mdns.dir/build: mdns/libmdns.a

.PHONY : mdns/CMakeFiles/mdns.dir/build

mdns/CMakeFiles/mdns.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/mdns && $(CMAKE_COMMAND) -P CMakeFiles/mdns.dir/cmake_clean.cmake
.PHONY : mdns/CMakeFiles/mdns.dir/clean

mdns/CMakeFiles/mdns.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/mdns /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/mdns /home/milan/Desktop/BP/station/cmake-build-debug/mdns/CMakeFiles/mdns.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mdns/CMakeFiles/mdns.dir/depend
