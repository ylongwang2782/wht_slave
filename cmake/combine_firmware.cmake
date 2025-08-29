# 首先检查 srec_cat 是否可用
find_program(SREC_CAT_EXECUTABLE srec_cat)
if(NOT SREC_CAT_EXECUTABLE)
  message(FATAL_ERROR "srec_cat not found! Please install SRecord tools.")
endif()

# 使用 srec_cat 合并 bootloader 和应用程序 HEX 文件生成合并的 HEX 文件
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.hex
  COMMAND
    ${SREC_CAT_EXECUTABLE}
    ${PROJECT_SOURCE_DIR}/bootloader/GD32F4xx_Bootloader.hex -Intel
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex -Intel -o
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.hex -Intel
  DEPENDS ${PROJECT_SOURCE_DIR}/bootloader/GD32F4xx_Bootloader.hex
          ${CMAKE_PROJECT_NAME}
  COMMENT "Combining bootloader and application HEX files using srec_cat")

# 使用 srec_cat 生成合并的二进制文件（指定地址范围避免生成过大文件）
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.bin
  COMMAND
    ${SREC_CAT_EXECUTABLE}
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.hex -Intel
    -offset -0x08000000
    -o ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.bin -Binary
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.hex
  COMMENT "Converting combined HEX to binary using srec_cat")

# 创建一个自定义目标来触发合并过程
add_custom_target(
  firmware_combine ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.hex
          ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_combined.bin)

# 输出信息
message(STATUS "srec_cat found: ${SREC_CAT_EXECUTABLE}")
message(STATUS "Combined firmware will be generated as:")
message(STATUS "  - ${CMAKE_PROJECT_NAME}_combined.hex")
message(STATUS "  - ${CMAKE_PROJECT_NAME}_combined.bin")
