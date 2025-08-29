cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once, and is not re-generated if converter is
# called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set(UWB_CHIP_TYPE "CX310")
set(RS485_DEBUG ON)

if(UWB_CHIP_TYPE STREQUAL "DW1000")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC UWB_CHIP_TYPE_DW1000)
    target_link_libraries(${CMAKE_PROJECT_NAME} dw1000)
elseif(UWB_CHIP_TYPE STREQUAL "CX310")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC UWB_CHIP_TYPE_CX310)
    target_link_libraries(${CMAKE_PROJECT_NAME} cx310)
else()
    message(FATAL_ERROR "Unsupported chip type: ${UWB_CHIP_TYPE}")
endif()

if(RS485_DEBUG)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC RS485_DEBUG)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC RS232_DEBUG)
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(User)
add_subdirectory(easylogger)
add_subdirectory(FreeRTOScpp)
add_subdirectory(protocol)
add_subdirectory(factory_test)
add_subdirectory(bootloader)

# Link directories setup
target_link_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE # Add user sources here
)

# Add include paths
target_include_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    easylogger
    FreeRTOScpp
    WhtsProtocol
    cx310
    # Add user defined libraries
)

# 设置cmake install指令，烧录程序
include(${PROJECT_SOURCE_DIR}/cmake/flash_by_install.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/create_bin.cmake)

# 获取 Git 标签作为版本号
execute_process(
  COMMAND git describe --tags --abbrev=0
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_TAG
  OUTPUT_STRIP_TRAILING_WHITESPACE)

# 提取 MAJOR, MINOR, PATCH
string(REGEX MATCH "([0-9]+)\\.([0-9]+)\\.([0-9]+)" VERSION_MATCH ${GIT_TAG})
if(VERSION_MATCH)
  string(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\1"
                       PROJECT_VERSION_MAJOR ${VERSION_MATCH})
  string(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\2"
                       PROJECT_VERSION_MINOR ${VERSION_MATCH})
  string(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\3"
                       PROJECT_VERSION_PATCH ${VERSION_MATCH})
else()
  message(WARNING "Invalid git tag: ${GIT_TAG}, using default version 0.0.0")
  set(PROJECT_VERSION_MAJOR 0)
  set(PROJECT_VERSION_MINOR 0)
  set(PROJECT_VERSION_PATCH 0)
endif()
set(VERSION_STRING
    "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
)
string(TIMESTAMP BUILD_DATE "%Y%m%d")

# 添加版本定义，可以在代码中使用
add_definitions(
  -DFIRMWARE_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
  -DFIRMWARE_VERSION_MINOR=${PROJECT_VERSION_MINOR}
  -DFIRMWARE_VERSION_PATCH=${PROJECT_VERSION_PATCH}
  -DFIRMWARE_VERSION_STRING="${VERSION_STRING}"
  -DFIRMWARE_BUILD_DATE="${BUILD_DATE}")

# 合并 bootloader 和应用程序固件
# 注意：必须在目标创建之后包含
if(CMAKE_C_COMPILER_ID MATCHES "GNU")
    include(${PROJECT_SOURCE_DIR}/cmake/combine_firmware.cmake)
endif()