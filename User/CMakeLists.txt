add_library(
  UserModule OBJECT
  # Accessory
  Accessory/button.cpp
  Accessory/LockController.cpp
  # App
  App/slave_app.cpp
  App/SlaveDevice.cpp
  App/Master2SlaveMessageHandlers.cpp
  # ContinuityCollector
  ContinuityCollector/ContinuityCollector.cpp
  # CX310
  CX310/uwb_interface.cpp
  # hptimer
  hptimer/hptimer.cpp
  # SlotManager
  SlotManager/SlotManager.cpp
  # UID Task
  Task/uwb_task.cpp)

target_compile_definitions(
  UserModule
  PUBLIC FIRMWARE_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
         FIRMWARE_VERSION_MINOR=${PROJECT_VERSION_MINOR}
         FIRMWARE_VERSION_PATCH=${PROJECT_VERSION_PATCH}
         FIRMWARE_VERSION_STRING="${VERSION_STRING}"
         FIRMWARE_BUILD_DATE="${BUILD_DATE}")

target_include_directories(
  UserModule
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
         Accessory/
         App/
         ContinuityCollector/
         CX310/
         hptimer/
         SlotManager/
         Task/
         uid/)

target_link_libraries(
  UserModule PUBLIC easylogger STM32_Drivers FreeRTOS FreeRTOScpp ProtocolCore
                    BootloaderModule)

if(UWB_CHIP_TYPE STREQUAL "DW1000")
  target_compile_definitions(UserModule PUBLIC UWB_CHIP_TYPE_DW1000)
elseif(UWB_CHIP_TYPE STREQUAL "CX310")
  target_compile_definitions(UserModule PUBLIC UWB_CHIP_TYPE_CX310)
else()
  message(FATAL_ERROR "Unsupported chip type: ${UWB_CHIP_TYPE}")
endif()
