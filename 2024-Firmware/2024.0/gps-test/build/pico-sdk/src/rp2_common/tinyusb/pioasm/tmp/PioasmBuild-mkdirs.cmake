# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/pico-sdk/tools/pioasm"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pioasm"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/tmp"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/gps-test/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()