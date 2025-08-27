# Create ContinuityCollector module library
add_library(ContinuityCollectorModule OBJECT
    ContinuityCollector.cpp
)

target_include_directories(ContinuityCollectorModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ContinuityCollectorModule PUBLIC easylogger HPTimerModule)

