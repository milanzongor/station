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
include heap/CMakeFiles/heap.dir/depend.make

# Include the progress variables for this target.
include heap/CMakeFiles/heap.dir/progress.make

# Include the compile flags for this target's objects.
include heap/CMakeFiles/heap.dir/flags.make

heap/CMakeFiles/heap.dir/heap_caps.c.obj: heap/CMakeFiles/heap.dir/flags.make
heap/CMakeFiles/heap.dir/heap_caps.c.obj: /home/milan/esp/esp-idf/components/heap/heap_caps.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object heap/CMakeFiles/heap.dir/heap_caps.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/heap.dir/heap_caps.c.obj   -c /home/milan/esp/esp-idf/components/heap/heap_caps.c

heap/CMakeFiles/heap.dir/heap_caps.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/heap.dir/heap_caps.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/heap/heap_caps.c > CMakeFiles/heap.dir/heap_caps.c.i

heap/CMakeFiles/heap.dir/heap_caps.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/heap.dir/heap_caps.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/heap/heap_caps.c -o CMakeFiles/heap.dir/heap_caps.c.s

heap/CMakeFiles/heap.dir/heap_caps_init.c.obj: heap/CMakeFiles/heap.dir/flags.make
heap/CMakeFiles/heap.dir/heap_caps_init.c.obj: /home/milan/esp/esp-idf/components/heap/heap_caps_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object heap/CMakeFiles/heap.dir/heap_caps_init.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/heap.dir/heap_caps_init.c.obj   -c /home/milan/esp/esp-idf/components/heap/heap_caps_init.c

heap/CMakeFiles/heap.dir/heap_caps_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/heap.dir/heap_caps_init.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/heap/heap_caps_init.c > CMakeFiles/heap.dir/heap_caps_init.c.i

heap/CMakeFiles/heap.dir/heap_caps_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/heap.dir/heap_caps_init.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/heap/heap_caps_init.c -o CMakeFiles/heap.dir/heap_caps_init.c.s

heap/CMakeFiles/heap.dir/heap_trace.c.obj: heap/CMakeFiles/heap.dir/flags.make
heap/CMakeFiles/heap.dir/heap_trace.c.obj: /home/milan/esp/esp-idf/components/heap/heap_trace.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object heap/CMakeFiles/heap.dir/heap_trace.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/heap.dir/heap_trace.c.obj   -c /home/milan/esp/esp-idf/components/heap/heap_trace.c

heap/CMakeFiles/heap.dir/heap_trace.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/heap.dir/heap_trace.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/heap/heap_trace.c > CMakeFiles/heap.dir/heap_trace.c.i

heap/CMakeFiles/heap.dir/heap_trace.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/heap.dir/heap_trace.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/heap/heap_trace.c -o CMakeFiles/heap.dir/heap_trace.c.s

heap/CMakeFiles/heap.dir/multi_heap.c.obj: heap/CMakeFiles/heap.dir/flags.make
heap/CMakeFiles/heap.dir/multi_heap.c.obj: /home/milan/esp/esp-idf/components/heap/multi_heap.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object heap/CMakeFiles/heap.dir/multi_heap.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/heap.dir/multi_heap.c.obj   -c /home/milan/esp/esp-idf/components/heap/multi_heap.c

heap/CMakeFiles/heap.dir/multi_heap.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/heap.dir/multi_heap.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/heap/multi_heap.c > CMakeFiles/heap.dir/multi_heap.c.i

heap/CMakeFiles/heap.dir/multi_heap.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/heap.dir/multi_heap.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/heap/multi_heap.c -o CMakeFiles/heap.dir/multi_heap.c.s

# Object files for target heap
heap_OBJECTS = \
"CMakeFiles/heap.dir/heap_caps.c.obj" \
"CMakeFiles/heap.dir/heap_caps_init.c.obj" \
"CMakeFiles/heap.dir/heap_trace.c.obj" \
"CMakeFiles/heap.dir/multi_heap.c.obj"

# External object files for target heap
heap_EXTERNAL_OBJECTS =

heap/libheap.a: heap/CMakeFiles/heap.dir/heap_caps.c.obj
heap/libheap.a: heap/CMakeFiles/heap.dir/heap_caps_init.c.obj
heap/libheap.a: heap/CMakeFiles/heap.dir/heap_trace.c.obj
heap/libheap.a: heap/CMakeFiles/heap.dir/multi_heap.c.obj
heap/libheap.a: heap/CMakeFiles/heap.dir/build.make
heap/libheap.a: heap/CMakeFiles/heap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libheap.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && $(CMAKE_COMMAND) -P CMakeFiles/heap.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/heap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
heap/CMakeFiles/heap.dir/build: heap/libheap.a

.PHONY : heap/CMakeFiles/heap.dir/build

heap/CMakeFiles/heap.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/heap && $(CMAKE_COMMAND) -P CMakeFiles/heap.dir/cmake_clean.cmake
.PHONY : heap/CMakeFiles/heap.dir/clean

heap/CMakeFiles/heap.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/heap /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/heap /home/milan/Desktop/BP/station/cmake-build-debug/heap/CMakeFiles/heap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : heap/CMakeFiles/heap.dir/depend

