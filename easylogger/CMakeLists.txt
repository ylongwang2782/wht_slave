add_library(
  easylogger OBJECT
  plugins/file/elog_file.c
  plugins/file/elog_file_port.c
  port/elog_port.c
  src/elog_async.c
  src/elog_buf.c
  src/elog_utils.c
  src/elog.c)

# Set include directories for this library
target_include_directories(
  easylogger
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/inc
         ${CMAKE_CURRENT_SOURCE_DIR}/plugins/file
         ${CMAKE_CURRENT_SOURCE_DIR}/port ${CMAKE_CURRENT_SOURCE_DIR}/src
         ${CMAKE_PROJECT_SOURCE_DIR}/Components/EasyLogger_Port
         
         )

target_link_libraries(easylogger PUBLIC FreeRTOS)
