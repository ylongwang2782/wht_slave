# Flash标志位实现说明

## 概述

本文档说明了将bootloader标志位从RAM存储改为Flash存储的实现方案，解决了RAM在软复位后被清除的问题。

## 技术方案

### Flash内存布局

STM32F429ZI的Flash分布：
- 总容量: 2MB (0x08000000 - 0x081FFFFF)
- Bootloader: 0x08000000 - 0x08007FFF (32KB)
- 应用程序: 0x08008000 - 0x081DFFFF (1888KB)
- **标志位区域: 0x081E0000 - 0x081FFFFF (Sector 23, 128KB)**

### 标志位存储位置

```c
#define BOOTLOADER_FLAG_SECTOR      23                    // Flash扇区23
#define BOOTLOADER_FLAG_ADDRESS     0x081FFF00UL          // 扇区23末尾预留256字节
#define BOOTLOADER_FLAG_MAGIC       0x12345678UL          // 魔数验证
#define BOOTLOADER_FLAG_UPGRADE     0xABCDEF00UL          // 升级标志
```

### 数据结构

```c
typedef struct {
    uint32_t magic;    // 魔数，用于验证结构的有效性
    uint32_t flag;     // 标志位，指示是否进入bootloader
} bootloader_flag_t;
```

## 实现细节

### Flash操作函数

#### 1. 擦除操作
```c
static HAL_StatusTypeDef flash_erase_flag_sector(void);
```
- 擦除整个Sector 23 (128KB)
- 使用HAL_FLASHEx_Erase()函数
- 电压范围: 2.7V-3.6V

#### 2. 写入操作
```c
static HAL_StatusTypeDef flash_write_flag_data(const bootloader_flag_t* flag_data);
```
- 按32位字写入magic和flag字段
- 使用HAL_FLASH_Program()函数
- 自动解锁/锁定Flash

#### 3. 读取操作
```c
static const bootloader_flag_t* get_bootloader_flag_ptr(void);
```
- 直接从Flash地址读取
- 无需特殊操作，Flash可以直接按内存访问

### 公共API

#### 设置标志位
```c
void set_bootloader_upgrade_flag(void);
```
1. 准备标志位数据结构
2. 擦除Flash扇区
3. 写入标志位数据
4. 验证写入结果

#### 清除标志位
```c
void clear_bootloader_flag(void);
```
- 擦除整个扇区即可清除所有数据
- Flash擦除后默认值为0xFFFFFFFF

#### 检查标志位
```c
uint8_t check_bootloader_upgrade_flag(void);
```
- 直接从Flash读取数据
- 验证magic和flag字段
- 返回布尔值

#### 触发复位
```c
void trigger_system_reset_to_bootloader(void);
```
1. 设置Flash标志位
2. 短暂延时确保写入完成
3. 执行NVIC_SystemReset()

## 内存保护

### 链接脚本修改

原始配置:
```
FLASH (rx) : ORIGIN = 0x08008000, LENGTH = 2016K
```

修改后:
```
FLASH (rx) : ORIGIN = 0x08008000, LENGTH = 1888K
```

这样确保应用程序不会使用Sector 23，避免意外覆盖标志位。

### Flash扇区保护

STM32F429的Flash扇区分布：
- Sector 0-11: 每个16KB或64KB (总512KB)
- Sector 12-22: 每个128KB (总1408KB) 
- **Sector 23: 128KB (专用于标志位)**

## 性能考虑

### 写入性能
- Flash擦除时间: ~100ms (128KB扇区)
- Flash写入时间: ~几ms (8字节数据)
- 总时间: ~100ms

### 读取性能
- Flash读取: 与RAM相同速度
- 无额外延时

### 擦写次数
- STM32F429 Flash: 10,000次擦写周期
- 对于偶尔的固件升级完全足够

## 与Bootloader的协作

### Bootloader需要实现

1. **启动时检查标志位**：
```c
if (check_bootloader_upgrade_flag()) {
    clear_bootloader_flag();  // 清除标志位防止循环
    enter_upgrade_mode();     // 进入升级模式
} else {
    jump_to_application();    // 跳转到应用程序
}
```

2. **标志位清除**：
- Bootloader必须在检查后立即清除标志位
- 避免无限循环进入升级模式

3. **Flash区域管理**：
- Bootloader升级应用程序时不要覆盖Sector 23
- 保持标志位区域的独立性

## 测试验证

### 基本功能测试

1. **写入测试**：
```bash
upgrade
```
检查Flash地址0x081FFF00是否写入正确的magic和flag值。

2. **复位保持测试**：
发送upgrade命令后，系统复位重启，检查标志位是否仍然存在。

3. **清除测试**：
验证clear_bootloader_flag()能正确擦除标志位。

### 工具验证

使用STM32CubeProgrammer等工具：
1. 读取地址0x081FFF00的8字节数据
2. 验证magic: 0x12345678
3. 验证flag: 0xABCDEF00

## 注意事项

1. **Flash写入限制**：
   - Flash只能从1写为0，不能从0写为1
   - 修改数据前必须先擦除扇区

2. **电源管理**：
   - Flash操作期间不能断电
   - 建议在写入过程中禁用看门狗

3. **并发保护**：
   - Flash操作不是原子的
   - 建议在设置标志位时禁用中断

4. **存储位置**：
   - 使用Sector 23末尾，为将来扩展留出空间
   - 256字节区域足够存储更多元数据

## 优势

相比RAM方案的优势：
1. **断电保持**：Flash内容在掉电后不丢失
2. **复位保持**：软复位后标志位依然有效
3. **可靠性高**：Flash写入后数据稳定
4. **调试友好**：可以用工具直接查看Flash内容

这个实现方案完全解决了RAM标志位在软复位后丢失的问题，提供了可靠的bootloader进入机制。
