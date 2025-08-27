# Create HPTimer module library
add_library(HPTimerModule OBJECT
    hptimer.cpp
)

target_include_directories(HPTimerModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(HPTimerModule PUBLIC STM32_Drivers)
