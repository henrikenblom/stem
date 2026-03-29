cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(stem_slave C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(stem_slave
    main.c
    stem_slave.c
)

pico_generate_pio_header(stem_slave ${CMAKE_CURRENT_LIST_DIR}/stem_slave.pio)

target_include_directories(stem_slave PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../protocol
)

target_link_libraries(stem_slave
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_clocks
    hardware_irq
)

pico_add_extra_outputs(stem_slave)
pico_enable_stdio_uart(stem_slave 1)
pico_enable_stdio_usb(stem_slave 0)
