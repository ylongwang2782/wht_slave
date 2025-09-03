#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <vector>

#include "ICX310.hpp"
#include "SemaphoreCPP.h"
#include "cmsis_os.h"
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

class CX310_SlaveSpiAdapter : public ICX310 {
   public:
    CX310_SlaveSpiAdapter();
    ~CX310_SlaveSpiAdapter();

   private:
    // 接收缓冲区
    uint8_t rx_buffer[1024];
    uint8_t dummy_data[1024];
    uint16_t recv_len;
    BinarySemaphore rx_semaphore = {"rx_semaphore"};
    long waswoken = 0;
    bool irq_enable = false;

    // HAL库SPI控制函数
    bool hal_spi_transmit(const std::vector<uint8_t>& data);
    bool hal_spi_receive(uint8_t* data, uint16_t size);

    // NSS控制函数
    void nss_low();
    void nss_high();

   public:
    void int_pin_irq_handler();

    // ICX310接口实现
    void reset_pin_init() override;
    void generate_reset_signal() override;
    void turn_of_reset_signal() override;
    bool send(std::vector<uint8_t>& tx_data) override;
    bool get_recv_data(std::queue<uint8_t>& rx_data) override;
    void commuication_peripheral_init() override;
    void chip_en_init() override;
    void chip_enable() override;
    void chip_disable() override;
    uint32_t get_system_1ms_ticks() override;
    void delay_ms(uint32_t ms) override;
    void log(const char* format, ...) override;
};

// 全局指针，用于在C中断函数中访问CX310_SlaveSpiAdapter实例
extern "C" {
    extern CX310_SlaveSpiAdapter* g_uwbAdapter;
}