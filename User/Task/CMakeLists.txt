# Create Task module library
add_library(TaskModule OBJECT
    uwb_task.cpp
)

target_include_directories(TaskModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(TaskModule PUBLIC FreeRTOS easylogger)

if(UWB_CHIP_TYPE STREQUAL "DW1000")
  target_compile_definitions(TaskModule PUBLIC UWB_CHIP_TYPE_DW1000)
  target_link_libraries(TaskModule PUBLIC dw1000)
elseif(UWB_CHIP_TYPE STREQUAL "CX310")
  target_compile_definitions(TaskModule PUBLIC UWB_CHIP_TYPE_CX310)
  target_link_libraries(TaskModule PUBLIC cx310)
else()
  message(FATAL_ERROR "Unsupported chip type: ${UWB_CHIP_TYPE}")
endif()