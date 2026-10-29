cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR
        "PICO_SDK_PATH is not set. Point it at the Raspberry Pi Pico SDK, for example:\n"
        "  export PICO_SDK_PATH=$HOME/pico/pico-sdk")
endif()

include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(mini_dummy LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(mini_dummy
    src/main.cpp
)

target_include_directories(mini_dummy PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_options(mini_dummy PRIVATE
    -Wall
    -Wextra
)

target_link_libraries(mini_dummy PRIVATE
    pico_stdlib
)

# Use the Pico SDK USB-CDC stdio transport. UART stdio is disabled so commands
# and responses have one unambiguous transport.
pico_enable_stdio_usb(mini_dummy 1)
pico_enable_stdio_uart(mini_dummy 0)

pico_add_extra_outputs(mini_dummy)
