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

# Utility rule file for ldgen.

# Include the progress variables for this target.
include CMakeFiles/ldgen.dir/progress.make

CMakeFiles/ldgen:


ldgen: CMakeFiles/ldgen
ldgen: CMakeFiles/ldgen.dir/build.make

.PHONY : ldgen

# Rule to build all files generated by this target.
CMakeFiles/ldgen.dir/build: ldgen

.PHONY : CMakeFiles/ldgen.dir/build

CMakeFiles/ldgen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ldgen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ldgen.dir/clean

CMakeFiles/ldgen.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/Desktop/BP/station /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles/ldgen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ldgen.dir/depend
