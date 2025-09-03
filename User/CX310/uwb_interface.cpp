#include "uwb_interface.hpp"

#include "elog.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

// 全局指针定义
CX310_SlaveSpiAdapter *g_uwbAdapter = nullptr;

// C风格包装函数，用于在C中断中调用C++成员函数
extern "C" void uwb_int_handler_wrapper(void)
{
    if (g_uwbAdapter != nullptr)
    {
        g_uwbAdapter->int_pin_irq_handler();
    }
}

// 构造函数
CX310_SlaveSpiAdapter::CX310_SlaveSpiAdapter()
{
    memset(dummy_data, 0x00, sizeof(dummy_data));
}

// 析构函数
CX310_SlaveSpiAdapter::~CX310_SlaveSpiAdapter()
{
}

// NSS控制函数实现
void CX310_SlaveSpiAdapter::nss_low()
{
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
}

void CX310_SlaveSpiAdapter::nss_high()
{
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
}

// 中断处理函数实现
void CX310_SlaveSpiAdapter::int_pin_irq_handler()
{
    if (!irq_enable)
    {
        return;
    }
    if (HAL_GPIO_ReadPin(UWB_INT_GPIO_Port, UWB_INT_Pin) == GPIO_PIN_RESET)
    {
        rx_semaphore.give_ISR(waswoken);
    }
}

void CX310_SlaveSpiAdapter::reset_pin_init()
{
    // 已在CubeMX中初始化并保证高电平
    HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_SET);
}

void CX310_SlaveSpiAdapter::generate_reset_signal()
{
    HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_RESET);
}

void CX310_SlaveSpiAdapter::turn_of_reset_signal()
{
    HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_SET);
}

bool CX310_SlaveSpiAdapter::send(std::vector<uint8_t> &tx_data)
{
    // 进入临界区，防止SPI通信被其他任务或中断打断
    taskENTER_CRITICAL();

    nss_low();

    while (HAL_GPIO_ReadPin(UWB_RDY_GPIO_Port, UWB_RDY_Pin) == GPIO_PIN_SET)
    {
    }

    bool ret = HAL_SPI_Transmit(&hspi4, const_cast<uint8_t *>(tx_data.data()), tx_data.size(), HAL_MAX_DELAY) == HAL_OK;
    nss_high();

    // 退出临界区
    taskEXIT_CRITICAL();

    return ret;
}

bool CX310_SlaveSpiAdapter::get_recv_data(std::queue<uint8_t> &rx_data)
{
    if (rx_semaphore.take(0))
    {
        nss_low();
        HAL_StatusTypeDef status;
        status = HAL_SPI_TransmitReceive(&hspi4, dummy_data, rx_buffer, 4, HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            nss_high();
            return false;
        }
        recv_len = (((uint16_t)rx_buffer[2]) << 8) | rx_buffer[3];
        // elog_i("IUWB", "recv_len: %d", recv_len);
        status = HAL_SPI_TransmitReceive(&hspi4, dummy_data, rx_buffer + 4, recv_len, HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            nss_high();
            return false;
        }
        nss_high();
        for (int i = 0; i < recv_len + 4; i++)
        {
            rx_data.push(rx_buffer[i]);
        }
        memset(rx_buffer, 0, sizeof(rx_buffer));
        return true;
    }
    return false;
}

void CX310_SlaveSpiAdapter::commuication_peripheral_init()
{
    irq_enable = true;
}

void CX310_SlaveSpiAdapter::chip_en_init()
{
    // 已在CubeMX中初始化，并保证高电平
    HAL_GPIO_WritePin(UWB_EN_GPIO_Port, UWB_EN_Pin, GPIO_PIN_SET);
}

void CX310_SlaveSpiAdapter::chip_enable()
{
    HAL_GPIO_WritePin(UWB_EN_GPIO_Port, UWB_EN_Pin, GPIO_PIN_SET);
}

void CX310_SlaveSpiAdapter::chip_disable()
{
    HAL_GPIO_WritePin(UWB_EN_GPIO_Port, UWB_EN_Pin, GPIO_PIN_RESET);
}

uint32_t CX310_SlaveSpiAdapter::get_system_1ms_ticks()
{
    return osKernelGetTickCount();
}

void CX310_SlaveSpiAdapter::delay_ms(uint32_t ms)
{
    osDelay(ms);
}

void CX310_SlaveSpiAdapter::log(const char *format, ...)
{
    // 空实现
}