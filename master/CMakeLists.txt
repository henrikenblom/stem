cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(stem_master C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(stem_master
    main.c
    stem_master.c
)

pico_generate_pio_header(stem_master ${CMAKE_CURRENT_LIST_DIR}/stem_master.pio)

target_include_directories(stem_master PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../protocol
)

target_link_libraries(stem_master
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_clocks
)

pico_add_extra_outputs(stem_master)
pico_enable_stdio_uart(stem_master 1)
pico_enable_stdio_usb(stem_master 0)
