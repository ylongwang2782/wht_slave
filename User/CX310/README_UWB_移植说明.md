# UWB接口移植说明

## 概述

本文件说明了如何将UWB接口从原有工程移植到基于STM32 HAL库的CubeMX工程中。

## 移植内容

### 1. 引脚定义

使用`main.h`中定义的引脚宏：

```c
// UWB相关引脚
#define UWB_RST_Pin GPIO_PIN_3
#define UWB_RST_GPIO_Port GPIOE

#define UWB_EN_Pin GPIO_PIN_2  
#define UWB_EN_GPIO_Port GPIOF

#define UWB_INT_Pin GPIO_PIN_6
#define UWB_INT_GPIO_Port GPIOB

#define UWB_RDY_Pin GPIO_PIN_7
#define UWB_RDY_GPIO_Port GPIOB

#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
```

### 2. 主要修改

#### 2.1 GPIO控制
- 替换原有的GPIO类为HAL库的GPIO函数
- 使用`HAL_GPIO_WritePin()`和`HAL_GPIO_ReadPin()`进行引脚控制
- 使用`HAL_GPIO_Init()`进行引脚初始化

#### 2.2 SPI通信
- 使用HAL库的SPI句柄`hspi4`
- 替换原有的SPI类为HAL库的SPI函数：
  - `HAL_SPI_Transmit()` - 发送数据
  - `HAL_SPI_Receive()` - 接收数据
  - `HAL_SPI_TransmitReceive()` - 收发数据

#### 2.3 外部中断
- 使用HAL库的GPIO中断模式`GPIO_MODE_IT_FALLING`
- 在`stm32f4xx_it.c`中添加`EXTI9_5_IRQHandler()`中断处理函数
- 使用全局变量传递适配器实例指针

### 3. 文件结构

```
User/CX310/
├── uwb_interface.hpp          # 移植后的UWB接口适配器
├── CX310.hpp                  # UWB设备类（未修改）
├── ICX310.hpp                 # UWB接口基类（未修改）
└── README_UWB_移植说明.md     # 本说明文件
```

### 4. 使用方法

```cpp
#include "uwb_interface.hpp"
#include "CX310.hpp"

// 创建UWB适配器
CX310_SlaveSpiAdapter adapter;

// 创建UWB设备
CX310<CX310_SlaveSpiAdapter> uwb_device(adapter);

// 初始化
adapter.reset_pin_init();
adapter.chip_en_init();
adapter.commuication_peripheral_init();
adapter.chip_enable();

// 初始化UWB设备
uwb_device.init();

// 使用UWB功能
uwb_device.set_channel(5);
uwb_device.data_transmit(data);
uwb_device.get_recv_data(received_data);
```

### 5. 关键特性

- **完全兼容原有接口**：保持`ICX310`接口不变
- **HAL库集成**：使用STM32 HAL库的GPIO、SPI和中断功能
- **中断处理**：支持外部中断处理UWB数据接收
- **线程安全**：使用FreeRTOS信号量进行线程间通信

### 6. 注意事项

1. **SPI配置**：确保CubeMX中SPI4已正确配置
2. **中断优先级**：UWB中断优先级设置为6
3. **引脚连接**：确保硬件连接与引脚定义一致
4. **时钟配置**：确保相关GPIO端口时钟已使能

### 7. 测试建议

1. 首先测试GPIO控制功能（复位、使能引脚）
2. 测试SPI通信功能
3. 测试中断处理功能
4. 最后测试完整的UWB通信功能

## 移植完成

移植后的UWB接口完全基于STM32 HAL库，可以在CubeMX生成的工程中正常使用。 