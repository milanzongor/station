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
include log/CMakeFiles/log.dir/depend.make

# Include the progress variables for this target.
include log/CMakeFiles/log.dir/progress.make

# Include the compile flags for this target's objects.
include log/CMakeFiles/log.dir/flags.make

log/CMakeFiles/log.dir/log.c.obj: log/CMakeFiles/log.dir/flags.make
log/CMakeFiles/log.dir/log.c.obj: /home/milan/esp/esp-idf/components/log/log.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object log/CMakeFiles/log.dir/log.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/log.dir/log.c.obj   -c /home/milan/esp/esp-idf/components/log/log.c

log/CMakeFiles/log.dir/log.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/log.dir/log.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/log/log.c > CMakeFiles/log.dir/log.c.i

log/CMakeFiles/log.dir/log.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/log.dir/log.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/log/log.c -o CMakeFiles/log.dir/log.c.s

# Object files for target log
log_OBJECTS = \
"CMakeFiles/log.dir/log.c.obj"

# External object files for target log
log_EXTERNAL_OBJECTS =

log/liblog.a: log/CMakeFiles/log.dir/log.c.obj
log/liblog.a: log/CMakeFiles/log.dir/build.make
log/liblog.a: log/CMakeFiles/log.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library liblog.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && $(CMAKE_COMMAND) -P CMakeFiles/log.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/log.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
log/CMakeFiles/log.dir/build: log/liblog.a

.PHONY : log/CMakeFiles/log.dir/build

log/CMakeFiles/log.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/log && $(CMAKE_COMMAND) -P CMakeFiles/log.dir/cmake_clean.cmake
.PHONY : log/CMakeFiles/log.dir/clean

log/CMakeFiles/log.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/log /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/log /home/milan/Desktop/BP/station/cmake-build-debug/log/CMakeFiles/log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : log/CMakeFiles/log.dir/depend

