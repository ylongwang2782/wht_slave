# Protocol Module CMakeLists.txt

# Create Protocol Core library
add_library(ProtocolCore OBJECT 
    DeviceStatus.cpp
    Frame.cpp
    ProtocolProcessor.cpp
    # messages
    messages/Master2Slave.cpp
    messages/Slave2Master.cpp
    messages/Backend2Master.cpp
    messages/Master2Backend.cpp
    messages/Slave2Backend.cpp
    #utils
    utils/ByteUtils.cpp
)

# Set include directories for ProtocolCore
target_include_directories(ProtocolCore 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/messages
    ${CMAKE_CURRENT_SOURCE_DIR}/../app
)

target_link_libraries(ProtocolCore PUBLIC easylogger)

# Set target properties
set_target_properties(ProtocolCore PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)