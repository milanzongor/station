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
include cxx/CMakeFiles/cxx.dir/depend.make

# Include the progress variables for this target.
include cxx/CMakeFiles/cxx.dir/progress.make

# Include the compile flags for this target's objects.
include cxx/CMakeFiles/cxx.dir/flags.make

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj: cxx/CMakeFiles/cxx.dir/flags.make
cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj: /home/milan/esp/esp-idf/components/cxx/cxx_exception_stubs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj -c /home/milan/esp/esp-idf/components/cxx/cxx_exception_stubs.cpp

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/milan/esp/esp-idf/components/cxx/cxx_exception_stubs.cpp > CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/milan/esp/esp-idf/components/cxx/cxx_exception_stubs.cpp -o CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj: cxx/CMakeFiles/cxx.dir/flags.make
cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj: /home/milan/esp/esp-idf/components/cxx/cxx_guards.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cxx.dir/cxx_guards.cpp.obj -c /home/milan/esp/esp-idf/components/cxx/cxx_guards.cpp

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cxx.dir/cxx_guards.cpp.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/milan/esp/esp-idf/components/cxx/cxx_guards.cpp > CMakeFiles/cxx.dir/cxx_guards.cpp.i

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cxx.dir/cxx_guards.cpp.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/milan/esp/esp-idf/components/cxx/cxx_guards.cpp -o CMakeFiles/cxx.dir/cxx_guards.cpp.s

# Object files for target cxx
cxx_OBJECTS = \
"CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj" \
"CMakeFiles/cxx.dir/cxx_guards.cpp.obj"

# External object files for target cxx
cxx_EXTERNAL_OBJECTS =

cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/build.make
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libcxx.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && $(CMAKE_COMMAND) -P CMakeFiles/cxx.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cxx.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cxx/CMakeFiles/cxx.dir/build: cxx/libcxx.a

.PHONY : cxx/CMakeFiles/cxx.dir/build

cxx/CMakeFiles/cxx.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/cxx && $(CMAKE_COMMAND) -P CMakeFiles/cxx.dir/cmake_clean.cmake
.PHONY : cxx/CMakeFiles/cxx.dir/clean

cxx/CMakeFiles/cxx.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/cxx /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/cxx /home/milan/Desktop/BP/station/cmake-build-debug/cxx/CMakeFiles/cxx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cxx/CMakeFiles/cxx.dir/depend
