cmake_minimum_required(VERSION 3.22)

# BSP (Board Support Package) static library

# Automatically discover all .c source files in the Src directory with CONFIGURE_DEPENDS
file(GLOB BSP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c
)

# Create static library
add_library(bsp_lib STATIC ${BSP_SOURCES})

# Public include directories for consumers of this library
target_include_directories(bsp_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Link with STM32CubeMX generated code (CMSIS, HAL drivers, etc.)
target_link_libraries(bsp_lib PUBLIC 
    stm32cubemx
    foc_lib
    m
)

# Note: POSITION_INDEPENDENT_CODE OFF for bare-metal (no dynamic linker).
# -fPIC on bare-metal can cause GOT-related issues with global variable access.
set_target_properties(bsp_lib PROPERTIES
    POSITION_INDEPENDENT_CODE OFF
)
