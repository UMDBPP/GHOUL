# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/pico-sdk/tools/elf2uf2"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/tmp"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/src/ELF2UF2Build-stamp"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/src"
  "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/michaelshipman/Developer/GHOUL/2024-Firmware/2024.0/flight/build/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()
