# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

#set(PICO_BOARD pico2w CACHE STRING "Board type")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picow_sdcard_play C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(./src build)

# Add executable. Default name is the project name, version 0.1
add_executable(picow_sdcard_play
    main.c
    hw_config.c
)

# .pio ファイルからヘッダーを生成
#pico_generate_pio_header(picow_sdcard_play ${CMAKE_CURRENT_LIST_DIR}/pdm_output.pio)
pico_generate_pio_header(picow_sdcard_play ${CMAKE_CURRENT_LIST_DIR}/wsd_player.pio)

# Can leave these off for silent mode:
# add_compile_definitions(USE_PRINTF USE_DBG_PRINTF)
add_compile_definitions(
    USE_PRINTF USE_DBG_PRINTF
    SD_SPI_NO_HIGH_FREQ
)

# Add the standard library and FatFS/SPI to the build
target_link_libraries(picow_sdcard_play
    pico_stdlib
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    pico_multicore
    hardware_pio
    hardware_spi
    hardware_gpio
    hardware_clocks
)

pico_set_program_name(picow_sdcard_play "picow_sdcard_play")
pico_set_program_version(picow_sdcard_play "0.1")

# Choose source and destination for standard input and output:
#   See 4.1. Serial input and output on Raspberry Pi Pico in Getting started with Raspberry Pi Pico (https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
#   and 2.7.1. Standard Input/Output (stdio) Support in Raspberry Pi Pico C/C++ SDK (https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf):
pico_enable_stdio_uart(picow_sdcard_play 0)
pico_enable_stdio_usb(picow_sdcard_play 1)

pico_add_extra_outputs(picow_sdcard_play)
