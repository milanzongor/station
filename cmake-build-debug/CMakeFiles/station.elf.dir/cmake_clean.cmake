file(REMOVE_RECURSE
  "config/sdkconfig.h"
  "config/sdkconfig.cmake"
  "station.map"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.map"
  "dummy_main_src.c"
  "CMakeFiles/station.elf.dir/dummy_main_src.c.obj"
  "station.elf.pdb"
  "station.elf"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/station.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
