# Create Accessory module library
add_library(AccessoryModule OBJECT
    button.cpp
    LockController.cpp
)

target_include_directories(AccessoryModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
