cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

add_executable(
        main
        main.cpp
        Maze_Traversal.cpp
        Maze_Traversal.h
        API.cpp
        API.h
        Drivetrain.cpp
        Drivetrain.h
        Motor.h
        Motor.cpp
        encoder.cpp
        encoder.h
    )

pico_add_extra_outputs(main)

# Pull in common dependencies
target_link_libraries(main pico_stdlib hardware_gpio hardware_pwm hardware_pio hardware_irq hardware_timer hardware_clocks)

# Enable serial
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)