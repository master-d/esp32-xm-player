# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/yi/git/3rdparty/esp-idf/components/bootloader/subproject"
  "/home/yi/git/esp32-xm-player/build/bootloader"
  "/home/yi/git/esp32-xm-player/build/bootloader-prefix"
  "/home/yi/git/esp32-xm-player/build/bootloader-prefix/tmp"
  "/home/yi/git/esp32-xm-player/build/bootloader-prefix/src/bootloader-stamp"
  "/home/yi/git/esp32-xm-player/build/bootloader-prefix/src"
  "/home/yi/git/esp32-xm-player/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/yi/git/esp32-xm-player/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/yi/git/esp32-xm-player/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
