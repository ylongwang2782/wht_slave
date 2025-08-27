# Create SlotManager module library
add_library(SlotManagerModule OBJECT
    SlotManager.cpp
)

target_include_directories(SlotManagerModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(SlotManagerModule PUBLIC easylogger HPTimerModule)