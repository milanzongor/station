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

# Utility rule file for partition_table.

# Include the progress variables for this target.
include partition_table/CMakeFiles/partition_table.dir/progress.make

partition_table/CMakeFiles/partition_table: partition_table/partition-table.bin


partition_table/partition-table.bin: /home/milan/esp/esp-idf/components/partition_table/partitions_singleapp.csv
partition_table/partition-table.bin: /home/milan/esp/esp-idf/components/partition_table/gen_esp32part.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating partition-table.bin"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/partition_table && python /home/milan/esp/esp-idf/components/partition_table/gen_esp32part.py -q --offset 0x8000 --disable-md5sum --flash-size 2MB /home/milan/esp/esp-idf/components/partition_table/partitions_singleapp.csv partition-table.bin

partition_table: partition_table/CMakeFiles/partition_table
partition_table: partition_table/partition-table.bin
partition_table: partition_table/CMakeFiles/partition_table.dir/build.make

.PHONY : partition_table

# Rule to build all files generated by this target.
partition_table/CMakeFiles/partition_table.dir/build: partition_table

.PHONY : partition_table/CMakeFiles/partition_table.dir/build

partition_table/CMakeFiles/partition_table.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/partition_table && $(CMAKE_COMMAND) -P CMakeFiles/partition_table.dir/cmake_clean.cmake
.PHONY : partition_table/CMakeFiles/partition_table.dir/clean

partition_table/CMakeFiles/partition_table.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/partition_table /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/partition_table /home/milan/Desktop/BP/station/cmake-build-debug/partition_table/CMakeFiles/partition_table.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : partition_table/CMakeFiles/partition_table.dir/depend

