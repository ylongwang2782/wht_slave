# Create Factory Test module library
add_library(FactoryTestModule OBJECT factory_test.c)

target_include_directories(FactoryTestModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(FactoryTestModule PUBLIC STM32_Drivers easylogger)

target_compile_definitions(FactoryTestModule
    PUBLIC
        FIRMWARE_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        FIRMWARE_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        FIRMWARE_VERSION_PATCH=${PROJECT_VERSION_PATCH}
        FIRMWARE_VERSION_STRING="${VERSION_STRING}"
        FIRMWARE_BUILD_DATE="${BUILD_DATE}"
)
