# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/ESP32/esp-idf/components/bootloader/subproject"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/tmp"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/src"
  "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/mende/Documents/ESP32_IDF/I2c-Oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
