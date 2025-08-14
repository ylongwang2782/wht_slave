# Bootloader升级功能测试指南

## 测试环境准备

### 硬件要求
- STM32F429ZI开发板
- USB转串口模块（连接到调试串口）
- PC端串口调试工具

### 软件要求
- 串口调试工具（如PuTTY、picocom、minicom等）
- 支持bootloader的固件版本

## 测试步骤

### 1. 连接硬件
1. 将USB转串口模块连接到STM32开发板的调试串口：
   - TX -> PA1 (UART4_RX)
   - RX -> PA0 (UART4_TX)
   - GND -> GND

2. 将开发板上电并烧录固件

### 2. 配置串口工具
```bash
# 使用picocom（Linux/macOS）
picocom -b 921600 /dev/ttyUSB0

# 使用PuTTY（Windows）
# 设置：Serial, 921600, 8-N-1
```

### 3. 基本功能测试

#### 3.1 启动信息验证
上电后应该看到类似输出：
```
UART Command Handler Ready
Type 'help' for available commands
```

#### 3.2 命令响应测试
```
help
Available commands:
  upgrade  - Enter bootloader mode for firmware upgrade
  version  - Show firmware version
  help     - Show this help message

version
Firmware Version: 1.0.0
Build Date: Jan 15 2025 14:30:22
MCU: STM32F429ZI
```

#### 3.3 无效命令测试
```
test123
Unknown command: 'test123'. Type 'help' for available commands.
```

### 4. 升级功能测试

#### 4.1 正常升级流程
```
upgrade
Upgrading firmware... System will reset to bootloader mode.
[系统应该立即复位]
```

#### 4.2 验证标志位设置
在发送upgrade命令后，系统会：
1. 设置RAM末端的bootloader标志位
2. 执行软复位
3. Bootloader应该检测到标志位并进入升级模式

### 5. 调试和验证

#### 5.1 日志输出验证
确保以下日志正常输出：
- `UART command handler initialized successfully`
- `Setting bootloader upgrade flag...`
- `Flash sector 23 erased successfully`
- `Bootloader flag written to Flash at 0x081FFF00`
- `Bootloader upgrade flag set successfully at address 0x081FFF00`
- `Triggering system reset to bootloader...`

#### 5.2 Flash检查
可以通过调试器或STM32CubeProgrammer验证标志位是否正确设置在Flash地址0x081FFF00：
- Magic: 0x12345678
- Flag: 0xABCDEF00

使用STM32CubeProgrammer：
1. 连接到目标设备
2. 读取Flash地址0x081FFF00-0x081FFF07 (8字节)
3. 验证数据内容

## 常见问题排查

### 1. 串口无响应
- 检查波特率设置（921600）
- 确认TX/RX连接正确
- 验证固件是否正确烧录

### 2. 命令无法识别
- 确认发送回车符（\r或\n）
- 检查命令拼写
- 验证串口发送格式

### 3. 升级命令无效果
- 确认bootloader是否支持Flash标志位检查
- 验证Flash地址0x081FFF00是否正确写入
- 检查Flash擦除/写入是否成功
- 确认系统复位是否正常执行

### 4. 系统启动异常
- 检查向量偏移设置（0x08008000）
- 确认Flash地址配置正确
- 验证bootloader和应用程序地址分配

## 性能测试

### 响应时间
- 命令处理响应：< 10ms
- Flash擦除时间：~100ms
- Flash写入时间：~几ms
- 升级触发到复位：~110ms

### 内存使用
- 标志位占用：8字节（Flash Sector 23）
- Flash保留：128KB（整个Sector 23）
- 任务堆栈：1024字节
- 命令缓冲区：64字节

## 预期结果

正常工作的系统应该：
1. 启动时显示欢迎信息
2. 正确响应help、version命令
3. upgrade命令触发系统复位
4. Bootloader能够检测到升级标志位
5. 无内存泄漏或栈溢出现象

## 注意事项

1. 测试过程中保持串口连接稳定
2. 如果系统复位后无响应，可能需要手动重新连接串口
3. Bootloader必须正确实现标志位检查和清除逻辑
4. 测试前确保备份原始固件
