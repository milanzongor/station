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

# Utility rule file for app_trace_sections_info.

# Include the progress variables for this target.
include app_trace/CMakeFiles/app_trace_sections_info.dir/progress.make

app_trace/CMakeFiles/app_trace_sections_info: app_trace/app_trace.sections_info


app_trace/app_trace.sections_info: app_trace/libapp_trace.a
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating app_trace.sections_info"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/app_trace && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-objdump /home/milan/Desktop/BP/station/cmake-build-debug/app_trace/libapp_trace.a -h > /home/milan/Desktop/BP/station/cmake-build-debug/app_trace/app_trace.sections_info

app_trace_sections_info: app_trace/CMakeFiles/app_trace_sections_info
app_trace_sections_info: app_trace/app_trace.sections_info
app_trace_sections_info: app_trace/CMakeFiles/app_trace_sections_info.dir/build.make

.PHONY : app_trace_sections_info

# Rule to build all files generated by this target.
app_trace/CMakeFiles/app_trace_sections_info.dir/build: app_trace_sections_info

.PHONY : app_trace/CMakeFiles/app_trace_sections_info.dir/build

app_trace/CMakeFiles/app_trace_sections_info.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/app_trace && $(CMAKE_COMMAND) -P CMakeFiles/app_trace_sections_info.dir/cmake_clean.cmake
.PHONY : app_trace/CMakeFiles/app_trace_sections_info.dir/clean

app_trace/CMakeFiles/app_trace_sections_info.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/app_trace /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/app_trace /home/milan/Desktop/BP/station/cmake-build-debug/app_trace/CMakeFiles/app_trace_sections_info.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app_trace/CMakeFiles/app_trace_sections_info.dir/depend

