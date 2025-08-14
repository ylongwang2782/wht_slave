# 定义自定义构建目标：下载到目标单片机
set(OPENOCD_EXECUTABLE "openocd")

# install(CODE CODE "MESSAGE(\"Flash......\")" CODE "execute_process(COMMAND
# ${OPENOCD_EXECUTABLE} -f
# ${PROJECT_SOURCE_DIR}/Scripts/OpenOCD/openocd_gdlink.cfg -c \"init; reset
# halt; program ${EXECUTABLE_OUTPUT_PATH}/${EXECUTABLE_NAME}.elf verify reset\"
# -c shutdown)" )

# # install by elf install( CODE CODE "MESSAGE(\"Flash......\")" CODE
# "execute_process(COMMAND ${OPENOCD_EXECUTABLE} -f
# ${PROJECT_SOURCE_DIR}/Scripts/OpenOCD/openocd_gdlink.cfg -c \"init; reset
# halt; program ${EXECUTABLE_OUTPUT_PATH}/${EXECUTABLE_NAME}.elf reset\" -c
# shutdown)")

message(STATUS "Building SLAVE")
install(
  CODE CODE
  "MESSAGE(\"Flash SLAVE......\")"
  CODE "execute_process(COMMAND openocd -f
${PROJECT_SOURCE_DIR}/Scripts/OpenOCD/openocd_gdlink.cfg -c \"init; reset
halt; program ${PROJECT_SOURCE_DIR}/build/debug/wht_slave.elf reset\" -c
shutdown)")

# # install by bin install( CODE CODE "MESSAGE(\"Flash......\")" CODE
# "execute_process(COMMAND ${OPENOCD_EXECUTABLE} -f
# ${PROJECT_SOURCE_DIR}/Scripts/OpenOCD/openocd_gdlink.cfg -c \"init; reset
# halt; flash write_image erase
# $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>/${EXECUTABLE_NAME}.bin 0x08008000 bin;
# reset run\" -c shutdown)" )

# install by jlink install( CODE CODE "MESSAGE(\"Flash......\")" CODE
# "execute_process(COMMAND ${OPENOCD_EXECUTABLE} -f
# ${PROJECT_SOURCE_DIR}/Scripts/OpenOCD/openocd_jlink.cfg -c \"init; reset halt;
# program ${EXECUTABLE_OUTPUT_PATH}/${EXECUTABLE_NAME}.hex reset\" -c
# shutdown)")
