# Create Bootloader module library
add_library(BootloaderModule OBJECT
    bootloader_flag.c
    uart_cmd_handler.c
)

target_include_directories(BootloaderModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(BootloaderModule PUBLIC STM32_Drivers FreeRTOS FactoryTestModule)
