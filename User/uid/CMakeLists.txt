# 创建纯头文件的接口库（INTERFACE library）
add_library(UIDModule INTERFACE)

# 指定头文件目录（公开给依赖此库的目标）
target_include_directories(UIDModule INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})