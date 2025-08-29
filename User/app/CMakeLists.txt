target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slave_app.cpp
          # ${CMAKE_CURRENT_SOURCE_DIR}/LockController.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/slave_device.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/master_slave_message_handlers.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
