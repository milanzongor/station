file(REMOVE_RECURSE
  "config/sdkconfig.h"
  "config/sdkconfig.cmake"
  "station.map"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.map"
  "CMakeFiles/size"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/size.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
