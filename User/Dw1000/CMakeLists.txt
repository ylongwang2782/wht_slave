# DW1000 Library CMakeLists.txt

# Create HAL DW1000 library
add_library(dw1000 OBJECT
    ./decadriver/deca_device.c
    ./platform/deca_mutex.c
    ./platform/deca_range_tables.c
    ./platform/deca_sleep.c
    ./platform/deca_spi.c
    ./platform/port.c
    ./decadriver/deca_params_init.c
)

# Set include directories for this library
target_include_directories(dw1000 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/decadriver
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
)