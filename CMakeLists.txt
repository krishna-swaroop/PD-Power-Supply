cmake_minimum_required(VERSION 3.19)

# Include the SDK CMake File
include(pico_sdk_import.cmake)

include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
# include(FreeRTOS_Kernel_import.cmake)

# Set Project Name
project(USBPD_Power_Supply C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(USBPD_Power_Supply
        src/main.c
        src/TPS55289.c 
)

# add_library(pindefinitions STATIC
#         include/pindefinitions.h)

target_include_directories(USBPD_Power_Supply PUBLIC
        include/
)


target_link_libraries(USBPD_Power_Supply
        pico_stdlib
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        # pindefinitions
)       

pico_add_extra_outputs(USBPD_Power_Supply)

pico_enable_stdio_usb(USBPD_Power_Supply 1)
pico_enable_stdio_uart(USBPD_Power_Supply 1)


