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
include ethernet/CMakeFiles/ethernet.dir/depend.make

# Include the progress variables for this target.
include ethernet/CMakeFiles/ethernet.dir/progress.make

# Include the compile flags for this target's objects.
include ethernet/CMakeFiles/ethernet.dir/flags.make

ethernet/CMakeFiles/ethernet.dir/emac_dev.c.obj: ethernet/CMakeFiles/ethernet.dir/flags.make
ethernet/CMakeFiles/ethernet.dir/emac_dev.c.obj: /home/milan/esp/esp-idf/components/ethernet/emac_dev.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ethernet/CMakeFiles/ethernet.dir/emac_dev.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ethernet.dir/emac_dev.c.obj   -c /home/milan/esp/esp-idf/components/ethernet/emac_dev.c

ethernet/CMakeFiles/ethernet.dir/emac_dev.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ethernet.dir/emac_dev.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/ethernet/emac_dev.c > CMakeFiles/ethernet.dir/emac_dev.c.i

ethernet/CMakeFiles/ethernet.dir/emac_dev.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ethernet.dir/emac_dev.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/ethernet/emac_dev.c -o CMakeFiles/ethernet.dir/emac_dev.c.s

ethernet/CMakeFiles/ethernet.dir/emac_main.c.obj: ethernet/CMakeFiles/ethernet.dir/flags.make
ethernet/CMakeFiles/ethernet.dir/emac_main.c.obj: /home/milan/esp/esp-idf/components/ethernet/emac_main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object ethernet/CMakeFiles/ethernet.dir/emac_main.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ethernet.dir/emac_main.c.obj   -c /home/milan/esp/esp-idf/components/ethernet/emac_main.c

ethernet/CMakeFiles/ethernet.dir/emac_main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ethernet.dir/emac_main.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/ethernet/emac_main.c > CMakeFiles/ethernet.dir/emac_main.c.i

ethernet/CMakeFiles/ethernet.dir/emac_main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ethernet.dir/emac_main.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/ethernet/emac_main.c -o CMakeFiles/ethernet.dir/emac_main.c.s

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj: ethernet/CMakeFiles/ethernet.dir/flags.make
ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj: /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_common.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj   -c /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_common.c

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ethernet.dir/eth_phy/phy_common.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_common.c > CMakeFiles/ethernet.dir/eth_phy/phy_common.c.i

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ethernet.dir/eth_phy/phy_common.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_common.c -o CMakeFiles/ethernet.dir/eth_phy/phy_common.c.s

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj: ethernet/CMakeFiles/ethernet.dir/flags.make
ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj: /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_lan8720.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj   -c /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_lan8720.c

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_lan8720.c > CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.i

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_lan8720.c -o CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.s

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj: ethernet/CMakeFiles/ethernet.dir/flags.make
ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj: /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_tlk110.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj   -c /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_tlk110.c

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.i"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_tlk110.c > CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.i

ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.s"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && /home/milan/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/milan/esp/esp-idf/components/ethernet/eth_phy/phy_tlk110.c -o CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.s

# Object files for target ethernet
ethernet_OBJECTS = \
"CMakeFiles/ethernet.dir/emac_dev.c.obj" \
"CMakeFiles/ethernet.dir/emac_main.c.obj" \
"CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj" \
"CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj" \
"CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj"

# External object files for target ethernet
ethernet_EXTERNAL_OBJECTS =

ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/emac_dev.c.obj
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/emac_main.c.obj
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_common.c.obj
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_lan8720.c.obj
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/eth_phy/phy_tlk110.c.obj
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/build.make
ethernet/libethernet.a: ethernet/CMakeFiles/ethernet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/milan/Desktop/BP/station/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C static library libethernet.a"
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && $(CMAKE_COMMAND) -P CMakeFiles/ethernet.dir/cmake_clean_target.cmake
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ethernet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ethernet/CMakeFiles/ethernet.dir/build: ethernet/libethernet.a

.PHONY : ethernet/CMakeFiles/ethernet.dir/build

ethernet/CMakeFiles/ethernet.dir/clean:
	cd /home/milan/Desktop/BP/station/cmake-build-debug/ethernet && $(CMAKE_COMMAND) -P CMakeFiles/ethernet.dir/cmake_clean.cmake
.PHONY : ethernet/CMakeFiles/ethernet.dir/clean

ethernet/CMakeFiles/ethernet.dir/depend:
	cd /home/milan/Desktop/BP/station/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/milan/Desktop/BP/station /home/milan/esp/esp-idf/components/ethernet /home/milan/Desktop/BP/station/cmake-build-debug /home/milan/Desktop/BP/station/cmake-build-debug/ethernet /home/milan/Desktop/BP/station/cmake-build-debug/ethernet/CMakeFiles/ethernet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ethernet/CMakeFiles/ethernet.dir/depend

