cmake_minimum_required(VERSION 3.22)

# BSP (Board Support Package) static library

# Automatically discover all .c source files in the Src directory with CONFIGURE_DEPENDS
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c
)

# Create static library
add_library(app_lib STATIC ${APP_SOURCES})

# Public include directories for consumers of this library
target_include_directories(app_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Link with STM32CubeMX generated code (CMSIS, HAL drivers, etc.)
target_link_libraries(app_lib PUBLIC 
    stm32cubemx
    foc_lib
    bsp_lib
)

# Set position independent code for static library
set_target_properties(app_lib PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
