cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)
set(PICO_BOARD "pico")

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

#include(pico_extras_import_optional.cmake)

project(routeur_solaire C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


# Initialize the SDK
pico_sdk_init()


add_executable(app
        src/app.c
        src/modbus.c
        src/data.c
        src/ssd1306_i2c/ssd1306_i2c.c
        )

target_include_directories(app PRIVATE src src/ssd1306_i2c)

# pull in common dependencies
target_link_libraries(app pico_stdlib hardware_rtc hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(app)

# add url via pico_set_program_url
#example_auto_set_url(app)
