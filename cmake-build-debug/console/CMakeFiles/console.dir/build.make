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
include console/CMakeFiles/console.dir/depend.make

# Include the progress variables for this target.
include console/CMakeFiles/console.dir/progress.make

# Include the compile flags for this target's objects.
include console/CMakeFiles/console.dir/flags.make

console/CMakeFiles/console.dir/commands.c.obj: console/CMakeFiles/console.dir/flags.make
console/CMakeFiles/console.dir/commands.c.obj: /home/milan/esp/esp-idf/components/console/commands.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object console/CMakeFiles/console.dir/commands.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/console.dir/commands.c.obj   -c /home/milan/esp/esp-idf/components/console/commands.c

console/CMakeFiles/console.dir/commands.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/console.dir/commands.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/console/commands.c > CMakeFiles/console.dir/commands.c.i

console/CMakeFiles/console.dir/commands.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/console.dir/commands.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/console/commands.c -o CMakeFiles/console.dir/commands.c.s

console/CMakeFiles/console.dir/split_argv.c.obj: console/CMakeFiles/console.dir/flags.make
console/CMakeFiles/console.dir/split_argv.c.obj: /home/milan/esp/esp-idf/components/console/split_argv.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object console/CMakeFiles/console.dir/split_argv.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/console.dir/split_argv.c.obj   -c /home/milan/esp/esp-idf/components/console/split_argv.c

console/CMakeFiles/console.dir/split_argv.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/console.dir/split_argv.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/console/split_argv.c > CMakeFiles/console.dir/split_argv.c.i

console/CMakeFiles/console.dir/split_argv.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/console.dir/split_argv.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/console/split_argv.c -o CMakeFiles/console.dir/split_argv.c.s

console/CMakeFiles/console.dir/argtable3/argtable3.c.obj: console/CMakeFiles/console.dir/flags.make
console/CMakeFiles/console.dir/argtable3/argtable3.c.obj: /home/milan/esp/esp-idf/components/console/argtable3/argtable3.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object console/CMakeFiles/console.dir/argtable3/argtable3.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/console.dir/argtable3/argtable3.c.obj   -c /home/milan/esp/esp-idf/components/console/argtable3/argtable3.c

console/CMakeFiles/console.dir/argtable3/argtable3.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/console.dir/argtable3/argtable3.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/console/argtable3/argtable3.c > CMakeFiles/console.dir/argtable3/argtable3.c.i

console/CMakeFiles/console.dir/argtable3/argtable3.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/console.dir/argtable3/argtable3.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/console/argtable3/argtable3.c -o CMakeFiles/console.dir/argtable3/argtable3.c.s

console/CMakeFiles/console.dir/linenoise/linenoise.c.obj: console/CMakeFiles/console.dir/flags.make
console/CMakeFiles/console.dir/linenoise/linenoise.c.obj: /home/milan/esp/esp-idf/components/console/linenoise/linenoise.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object console/CMakeFiles/console.dir/linenoise/linenoise.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/console.dir/linenoise/linenoise.c.obj   -c /home/milan/esp/esp-idf/components/console/linenoise/linenoise.c

console/CMakeFiles/console.dir/linenoise/linenoise.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/console.dir/linenoise/linenoise.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/console/linenoise/linenoise.c > CMakeFiles/console.dir/linenoise/linenoise.c.i

console/CMakeFiles/console.dir/linenoise/linenoise.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/console.dir/linenoise/linenoise.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/console/linenoise/linenoise.c -o CMakeFiles/console.dir/linenoise/linenoise.c.s

# Object files for target console
console_OBJECTS = \
"CMakeFiles/console.dir/commands.c.obj" \
"CMakeFiles/console.dir/split_argv.c.obj" \
"CMakeFiles/console.dir/argtable3/argtable3.c.obj" \
"CMakeFiles/console.dir/linenoise/linenoise.c.obj"

# External object files for target console
console_EXTERNAL_OBJECTS =

console/libconsole.a: console/CMakeFiles/console.dir/commands.c.obj
console/libconsole.a: console/CMakeFiles/console.dir/split_argv.c.obj
console/libconsole.a: console/CMakeFiles/console.dir/argtable3/argtable3.c.obj
console/libconsole.a: console/CMakeFiles/console.dir/linenoise/linenoise.c.obj
console/libconsole.a: console/CMakeFiles/console.dir/build.make
console/libconsole.a: console/CMakeFiles/console.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libconsole.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && $(CMAKE_COMMAND) -P CMakeFiles/console.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/console.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
console/CMakeFiles/console.dir/build: console/libconsole.a

.PHONY : console/CMakeFiles/console.dir/build

console/CMakeFiles/console.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/console && $(CMAKE_COMMAND) -P CMakeFiles/console.dir/cmake_clean.cmake
.PHONY : console/CMakeFiles/console.dir/clean

console/CMakeFiles/console.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/console /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/console /home/milan/Desktop/BP/station/cmake-build-debug/console/CMakeFiles/console.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : console/CMakeFiles/console.dir/depend
