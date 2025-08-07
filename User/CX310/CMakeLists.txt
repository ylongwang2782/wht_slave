# CX310 Library CMakeLists.txt

# Create HAL CX310 library
add_library(cx310 STATIC)

# Set include directories for this library
target_include_directories(cx310 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../)

# Add UWB interface source files
target_sources(cx310 PRIVATE uwb_interface.hpp uwb_interface.cpp)

target_link_libraries(cx310 PUBLIC stm32cubemx easylogger FreeRTOScpp)
