# Create App module library
add_library(
  AppModule OBJECT
  slave_app.cpp
  # LockController.cpp
  SlaveDevice.cpp Master2SlaveMessageHandlers.cpp)

target_include_directories(AppModule PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
  AppModule
  PUBLIC easylogger
         ContinuityCollectorModule
         ProtocolCore
         AccessoryModule
         SlotManagerModule
         FreeRTOScpp
         UIDModule
         TaskModule
         BootloaderModule
         )
