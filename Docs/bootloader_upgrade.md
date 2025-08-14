# Bootloader升级功能使用说明

## 功能概述

本固件实现了通过软件复位进入bootloader的功能，可以在运行时通过调试串口发送升级指令来触发系统复位并进入bootloader模式，避免了传统的按键+重启方式。

## 实现原理

### 内存布局
- STM32F429ZI 总RAM: 704KB (0x20000000 - 0x200B0000)
- Bootloader标志位地址: 0x200AFFF8 (RAM末端的最后8字节)
- 标志位结构:
  ```c
  typedef struct {
      uint32_t magic;     // 魔数: 0x12345678
      uint32_t flag;      // 升级标志: 0xABCDEF00
  } bootloader_flag_t;
  ```

### 工作流程
1. 应用程序通过调试串口接收"upgrade"命令
2. 设置bootloader标志位到RAM末端
3. 调用`NVIC_SystemReset()`执行软复位
4. Bootloader启动时检查标志位，如果存在则进入升级模式
5. Bootloader清除标志位，防止死循环

## 使用方法

### 1. 串口配置
- 调试串口: UART4 (PA0-TX, PA1-RX)
- 波特率: 921600
- 数据位: 8
- 停止位: 1
- 校验位: 无

### 2. 可用命令
连接调试串口后，可以发送以下命令：

#### 升级命令
```
upgrade
```
发送此命令后，系统会立即复位并尝试进入bootloader模式。

#### 帮助命令
```
help
```
显示所有可用命令的帮助信息。

#### 版本信息
```
version
```
显示当前固件版本和编译信息。

### 3. 使用示例
```
$ picocom -b 921600 /dev/ttyUSB0

UART Command Handler Ready
Type 'help' for available commands

help
Available commands:
  upgrade  - Enter bootloader mode for firmware upgrade
  version  - Show firmware version
  help     - Show this help message

version
Firmware Version: 1.0.0
Build Date: Jan 15 2025 14:30:22
MCU: STM32F429ZI

upgrade
Upgrading firmware... System will reset to bootloader mode.
[系统复位]
```

## 技术细节

### 关键函数
- `set_bootloader_upgrade_flag()`: 设置升级标志位
- `check_bootloader_upgrade_flag()`: 检查标志位状态
- `clear_bootloader_flag()`: 清除标志位
- `trigger_system_reset_to_bootloader()`: 触发复位进入bootloader

### 内存安全
- 标志位位于RAM末端，不会与应用程序数据冲突
- 使用魔数验证标志位的有效性
- 在main函数中会检查标志位状态（用于调试）

### 错误处理
- 串口接收支持退格键处理
- 命令缓冲区溢出保护
- 未知命令提示

## 注意事项

1. **Bootloader要求**: 此功能需要配套的bootloader支持标志位检查
2. **RAM共享**: 标志位区域需要在bootloader和应用程序之间共享
3. **标志位清除**: Bootloader必须在检查后清除标志位，防止死循环
4. **向量偏移**: 应用程序使用0x08008000作为起始地址（已设置向量偏移）

## 配置文件
相关的配置定义在以下文件中：
- `Core/Inc/bootloader_flag.h`: 标志位定义和函数声明
- `Core/Inc/uart_cmd_handler.h`: UART命令处理定义
- `STM32F429XX_FLASH.ld`: Flash和RAM内存布局

## 编译集成
新增的源文件已自动集成到CMake构建系统中：
- `Core/Src/bootloader_flag.c`
- `Core/Src/uart_cmd_handler.c`
