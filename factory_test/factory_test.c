/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : factory_test.c
 * @brief          : Factory test protocol implementation
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "factory_test.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "elog.h"
#include "main.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "usart.h"

/* Private variables ---------------------------------------------------------*/
static const char* TAG = "factory_test";
static factory_test_state_t test_state = FACTORY_TEST_DISABLED;

/* Factory test entry command definition */
const uint8_t FACTORY_TEST_ENTRY_CMD[FACTORY_TEST_ENTRY_CMD_LEN] = {
    0x55, 0xAA, 0x01, 0x02, 0x21, 0x00, 0x00, 0x48, 0x72, 0xBB, 0x66};

/* Factory test entry detection variables */
static uint8_t entry_cmd_buffer[FACTORY_TEST_ENTRY_CMD_LEN];
static uint8_t entry_cmd_index = 0;
static uint32_t detection_start_time = 0;
static bool entry_detection_active = false;

/* Ring buffer for interrupt-safe data transfer */
#define RING_BUFFER_SIZE 256
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static volatile uint16_t ring_head =
    0;    // Write pointer (updated in interrupt)
static volatile uint16_t ring_tail = 0;    // Read pointer (updated in task)

/* Frame processing buffer */
static uint8_t frame_buffer[FACTORY_TEST_BUFFER_SIZE];
static uint16_t frame_index = 0;

/* 64-way IO pin mapping table */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_pin_map_t;

static const gpio_pin_map_t io_pin_map[64] = {
    {IO1_GPIO_Port, IO1_Pin},   {IO2_GPIO_Port, IO2_Pin},
    {IO3_GPIO_Port, IO3_Pin},   {IO4_GPIO_Port, IO4_Pin},
    {IO5_GPIO_Port, IO5_Pin},   {IO6_GPIO_Port, IO6_Pin},
    {IO7_GPIO_Port, IO7_Pin},   {IO8_GPIO_Port, IO8_Pin},
    {IO9_GPIO_Port, IO9_Pin},   {IO10_GPIO_Port, IO10_Pin},
    {IO11_GPIO_Port, IO11_Pin}, {IO12_GPIO_Port, IO12_Pin},
    {IO13_GPIO_Port, IO13_Pin}, {IO14_GPIO_Port, IO14_Pin},
    {IO15_GPIO_Port, IO15_Pin}, {IO16_GPIO_Port, IO16_Pin},
    {IO17_GPIO_Port, IO17_Pin}, {IO18_GPIO_Port, IO18_Pin},
    {IO19_GPIO_Port, IO19_Pin}, {IO20_GPIO_Port, IO20_Pin},
    {IO21_GPIO_Port, IO21_Pin}, {IO22_GPIO_Port, IO22_Pin},
    {IO23_GPIO_Port, IO23_Pin}, {IO24_GPIO_Port, IO24_Pin},
    {IO25_GPIO_Port, IO25_Pin}, {IO26_GPIO_Port, IO26_Pin},
    {IO27_GPIO_Port, IO27_Pin}, {IO28_GPIO_Port, IO28_Pin},
    {IO29_GPIO_Port, IO29_Pin}, {IO30_GPIO_Port, IO30_Pin},
    {IO31_GPIO_Port, IO31_Pin}, {IO32_GPIO_Port, IO32_Pin},
    {IO33_GPIO_Port, IO33_Pin}, {IO34_GPIO_Port, IO34_Pin},
    {IO35_GPIO_Port, IO35_Pin}, {IO36_GPIO_Port, IO36_Pin},
    {IO37_GPIO_Port, IO37_Pin}, {IO38_GPIO_Port, IO38_Pin},
    {IO39_GPIO_Port, IO39_Pin}, {IO40_GPIO_Port, IO40_Pin},
    {IO41_GPIO_Port, IO41_Pin}, {IO42_GPIO_Port, IO42_Pin},
    {IO43_GPIO_Port, IO43_Pin}, {IO44_GPIO_Port, IO44_Pin},
    {IO45_GPIO_Port, IO45_Pin}, {IO46_GPIO_Port, IO46_Pin},
    {IO47_GPIO_Port, IO47_Pin}, {IO48_GPIO_Port, IO48_Pin},
    {IO49_GPIO_Port, IO49_Pin}, {IO50_GPIO_Port, IO50_Pin},
    {IO51_GPIO_Port, IO51_Pin}, {IO52_GPIO_Port, IO52_Pin},
    {IO53_GPIO_Port, IO53_Pin}, {IO54_GPIO_Port, IO54_Pin},
    {IO55_GPIO_Port, IO55_Pin}, {IO56_GPIO_Port, IO56_Pin},
    {IO57_GPIO_Port, IO57_Pin}, {IO58_GPIO_Port, IO58_Pin},
    {IO59_GPIO_Port, IO59_Pin}, {IO60_GPIO_Port, IO60_Pin},
    {IO61_GPIO_Port, IO61_Pin}, {IO62_GPIO_Port, IO62_Pin},
    {IO63_GPIO_Port, IO63_Pin}, {IO64_GPIO_Port, IO64_Pin}};

/* DIP switch pin mapping table */
static const gpio_pin_map_t dip_pin_map[8] = {
    {DIP1_GPIO_Port, DIP1_Pin}, {DIP2_GPIO_Port, DIP2_Pin},
    {DIP3_GPIO_Port, DIP3_Pin}, {DIP4_GPIO_Port, DIP4_Pin},
    {DIP5_GPIO_Port, DIP5_Pin}, {DIP6_GPIO_Port, DIP6_Pin},
    {DIP7_GPIO_Port, DIP7_Pin}, {DIP8_GPIO_Port, DIP8_Pin}};

/* CRC16-MODBUS lookup table */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601,
    0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0,
    0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81,
    0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0,
    0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081,
    0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00,
    0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700,
    0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
    0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01,
    0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1,
    0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101,
    0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481,
    0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
    0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581,
    0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040};

/* Private function prototypes -----------------------------------------------*/
static void factory_test_reset_frame_buffer(void);
static bool factory_test_find_frame_start(void);
static uint16_t factory_test_get_frame_length(const uint8_t* buffer);
static void factory_test_create_response_frame(factory_test_frame_t* response,
                                               uint8_t msg_id,
                                               const uint8_t* payload,
                                               uint16_t payload_len);

/* Ring buffer functions */
static bool ring_buffer_put(uint8_t data);
static bool ring_buffer_get(uint8_t* data);
static uint16_t ring_buffer_available(void);
static void ring_buffer_reset(void);

/* 64-way IO control functions */
static device_status_t io64_set_mode(uint64_t pin_mask, uint8_t mode);
static device_status_t io64_set_pull(uint64_t pin_mask, uint8_t pull);
static device_status_t io64_write_level(uint64_t pin_mask, uint8_t level);
static device_status_t io64_read_level(uint64_t* levels);

/* DIP switch control functions */
static device_status_t dip_read_level(uint8_t* levels);

/* GPIO configuration helper functions */
static uint32_t get_gpio_pin_mode(GPIO_TypeDef* port, uint16_t pin);
static uint32_t get_gpio_pin_pull(GPIO_TypeDef* port, uint16_t pin);

/* Public functions ----------------------------------------------------------*/

HAL_StatusTypeDef factory_test_send_data(const uint8_t* data, uint16_t length) {
    RS485_TX_EN();
    HAL_StatusTypeDef ret =
        HAL_UART_Transmit(&RS485_UART, data, length, HAL_MAX_DELAY);
    RS485_RX_EN();
    return ret;
}

/**
 * @brief  Initialize factory test mode
 * @retval None
 */
extern uint8_t uart_rx_char;
void factory_test_init(void) {
    // set elog level to error
    elog_set_filter_tag_lvl(TAG, ELOG_LVL_ERROR);

    RS485_RX_EN();
    HAL_UART_Receive_IT(&RS485_UART, &uart_rx_char, 1);

    test_state = FACTORY_TEST_DISABLED;
    ring_buffer_reset();
    factory_test_reset_frame_buffer();

    elog_i(TAG, "Factory test initialized");
}

/**
 * @brief  Start factory test entry detection
 * @retval None
 */
void factory_test_start_entry_detection(void) {
    entry_cmd_index = 0;
    detection_start_time = osKernelGetTickCount();
    entry_detection_active = true;
    memset(entry_cmd_buffer, 0, sizeof(entry_cmd_buffer));
    elog_i(TAG, "Started factory test entry detection for 1 second");
}

/**
 * @brief  Process byte for factory test entry detection
 * @param  data: Received byte
 * @retval true if entry command detected, false otherwise
 */
bool factory_test_process_entry_byte(uint8_t data) {
    if (!entry_detection_active) {
        return false;
    }

    // Check if detection timeout
    if ((osKernelGetTickCount() - detection_start_time) >
        FACTORY_TEST_DETECTION_TIMEOUT_MS) {
        entry_detection_active = false;
        elog_i(TAG, "Factory test entry detection timeout");
        return false;
    }

    // Store byte in buffer
    entry_cmd_buffer[entry_cmd_index] = data;
    entry_cmd_index++;

    // Check if we have enough bytes to compare
    if (entry_cmd_index >= FACTORY_TEST_ENTRY_CMD_LEN) {
        // Compare with expected command
        if (memcmp(entry_cmd_buffer, FACTORY_TEST_ENTRY_CMD,
                   FACTORY_TEST_ENTRY_CMD_LEN) == 0) {
            entry_detection_active = false;
            test_state = FACTORY_TEST_ENABLED;    // 设置工厂测试状态
            // elog_i(TAG, "Factory test entry command detected!");
            // reply with "OK"
            factory_test_frame_t response;
            uint8_t status = 0x00;    // OK
            factory_test_create_response_frame(&response, MSG_ID_ENTER_TEST,
                                               &status, 1);
            factory_test_send_response(&response);
            return true;
        } else {
            // Shift buffer left to continue detection
            memmove(entry_cmd_buffer, entry_cmd_buffer + 1,
                    FACTORY_TEST_ENTRY_CMD_LEN - 1);
            entry_cmd_index = FACTORY_TEST_ENTRY_CMD_LEN - 1;
        }
    }

    return false;
}

/**
 * @brief  Blocking check for factory test entry command within 1 second
 * @retval true if entry command detected, false otherwise
 */
bool factory_test_blocking_check_entry(void) {
    elog_i(TAG, "Starting blocking factory test entry detection (1 second)...");

    // 启动检测
    factory_test_start_entry_detection();

    // 阻塞等待1秒，每10ms检查一次
    uint32_t start_time = osKernelGetTickCount();
    uint32_t timeout_ticks = FACTORY_TEST_DETECTION_TIMEOUT_MS;

    while ((osKernelGetTickCount() - start_time) < timeout_ticks) {
        if (!entry_detection_active) {
            // 检测已完成（可能是检测到指令或其他原因）
            if (test_state == FACTORY_TEST_ENABLED) {
                elog_i(TAG, "Factory test entry command detected!");
                return true;
            }
            break;
        }
        osDelay(10);    // 10ms延迟避免过度占用CPU
    }

    // 超时或未检测到指令
    entry_detection_active = false;
    elog_i(TAG, "Factory test entry detection timeout - no command received");
    return false;
}

/**
 * @brief  Enter factory test mode
 * @retval None
 */
void factory_test_enter_mode(void) {
    ring_buffer_reset();
    factory_test_reset_frame_buffer();

    elog_i(TAG, "Entered factory test mode");
}

/**
 * @brief  Exit factory test mode
 * @retval None
 */
void factory_test_exit_mode(void) {
    test_state = FACTORY_TEST_DISABLED;
    ring_buffer_reset();
    factory_test_reset_frame_buffer();

    elog_i(TAG, "Exited factory test mode");
}

/**
 * @brief  Check if factory test mode is enabled
 * @retval true if enabled, false otherwise
 */
bool factory_test_is_enabled(void) {
    return (test_state != FACTORY_TEST_DISABLED);
}

/**
 * @brief  Store incoming data byte in ring buffer (called from interrupt)
 * @param  data: Received data byte
 * @retval None
 */
void factory_test_process_data(uint8_t data) {
    if (test_state != FACTORY_TEST_ENABLED) {
        return;    // Silently ignore data when not in test mode
    }

    // Store data in ring buffer (interrupt-safe)
    ring_buffer_put(data);
}

/**
 * @brief  Process frames from ring buffer (called from task)
 * @retval None
 */
void factory_test_task_process(void) {
    if (test_state != FACTORY_TEST_ENABLED) {
        return;
    }

    uint8_t data;

    // Process all available data from ring buffer
    while (ring_buffer_get(&data)) {
        // Store received byte in frame buffer
        if (frame_index < FACTORY_TEST_BUFFER_SIZE - 1) {
            frame_buffer[frame_index++] = data;
        } else {
            // Buffer overflow, reset
            factory_test_reset_frame_buffer();
            frame_buffer[frame_index++] = data;
        }

        // Try to find and parse a complete frame
        if (factory_test_find_frame_start()) {
            uint16_t frame_len = factory_test_get_frame_length(frame_buffer);

            if (frame_len > 0 && frame_index >= frame_len) {
                factory_test_frame_t frame;
                if (factory_test_parse_frame(frame_buffer, frame_len, &frame)) {
                    test_state = FACTORY_TEST_PROCESSING;

                    // Process the frame based on message ID
                    switch (frame.msg_id) {
                        case MSG_ID_HEARTBEAT:
                            elog_v(TAG, "heartbeat");
                            factory_test_handle_heartbeat(&frame);
                            break;
                        case MSG_ID_GPIO_CONTROL:
                            elog_v(TAG, "gpio_control");
                            factory_test_handle_gpio_control(&frame);
                            break;
                        case MSG_ID_64WAY_IO_CONTROL:
                            elog_v(TAG, "64way_io_control");
                            factory_test_handle_64way_io_control(&frame);
                            break;
                        case MSG_ID_DIP_SWITCH_CONTROL:
                            elog_v(TAG, "dip_switch_control");
                            factory_test_handle_dip_switch_control(&frame);
                            break;
                        case MSG_ID_ADDITIONAL_TEST:
                            elog_v(TAG, "execute_test");
                            factory_test_handle_additional_test(&frame);
                            break;
                        default:
                            elog_w(TAG, "Unknown message ID: 0x%02X",
                                   frame.msg_id);
                            break;
                    }

                    test_state = FACTORY_TEST_ENABLED;
                }

                // Remove processed frame from buffer
                if (frame_index > frame_len) {
                    memmove(frame_buffer, &frame_buffer[frame_len],
                            frame_index - frame_len);
                    frame_index -= frame_len;
                } else {
                    factory_test_reset_frame_buffer();
                }
            } else {
                // elog_v(TAG, "frame_index: %d, frame_len: %d", frame_index,
                //        frame_len);
            }
        }
    }
}

/**
 * @brief  Parse received frame
 * @param  buffer: Buffer containing frame data
 * @param  length: Length of frame data
 * @param  frame: Pointer to frame structure to fill
 * @retval true if frame parsed successfully, false otherwise
 */
bool factory_test_parse_frame(const uint8_t* buffer, uint16_t length,
                              factory_test_frame_t* frame) {
    if (length < 9) {    // Minimum frame size: SOF(2) + MSG(3) + LEN(2) +
                         // CRC(2) + EOF(2)
        elog_w(TAG, "length error");
        return false;
    }

    // Parse SOF (Little Endian)
    frame->sof = buffer[0] | (buffer[1] << 8);
    if (frame->sof != FACTORY_TEST_SOF) {
        elog_w(TAG, "sof error");
        return false;
    }

    // Parse message info
    frame->source = buffer[FRAME_SOURCE_OFFSET];
    frame->target = buffer[FRAME_TARGET_OFFSET];
    frame->msg_id = buffer[FRAME_MSG_ID_OFFSET];

    // Parse payload length (Little Endian)
    frame->payload_len = buffer[FRAME_PAYLOAD_LEN_OFFSET] |
                         (buffer[FRAME_PAYLOAD_LEN_OFFSET + 1] << 8);

    if (frame->payload_len > FACTORY_TEST_MAX_PAYLOAD) {
        elog_w(TAG, "payload_len error");
        return false;
    }

    // Check if we have enough data for the complete frame
    uint16_t expected_length = FRAME_PAYLOAD_OFFSET + frame->payload_len +
                               2;    // +2 for CRC(2) + EOF(2) - 2 for SOF(2)
    if (length < expected_length) {
        elog_w(TAG, "expected_length error");
        return false;
    }

    // Copy payload
    memcpy(frame->payload, &buffer[FRAME_PAYLOAD_OFFSET], frame->payload_len);

    // Parse CRC16 (Little Endian)
    uint16_t crc_offset = FRAME_PAYLOAD_OFFSET + frame->payload_len;
    frame->crc16 = buffer[crc_offset] | (buffer[crc_offset + 1] << 8);

    // Parse EOF (Little Endian)
    uint16_t eof_offset = crc_offset + 2;
    frame->eof = buffer[eof_offset] | (buffer[eof_offset + 1] << 8);
    if (frame->eof != FACTORY_TEST_EOF) {
        elog_w(TAG, "eof error");
        return false;
    }

    // Verify CRC16
    uint16_t calculated_crc = factory_test_calculate_crc16(
        &buffer[FRAME_SOURCE_OFFSET],
        3 + 2 + frame->payload_len);    // MSG(3) + LEN(2) + PAYLOAD
    if (calculated_crc != frame->crc16) {
        elog_w(TAG, "crc error, calculated_crc: %04X, frame->crc16: %04X",
               calculated_crc, frame->crc16);
        return false;
    }

    return true;
}

/**
 * @brief  Calculate CRC16-MODBUS
 * @param  data: Pointer to data
 * @param  length: Length of data
 * @retval CRC16 value
 */
uint16_t factory_test_calculate_crc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }

    return crc;
}

/**
 * @brief  Send response frame
 * @param  response: Pointer to response frame
 * @retval None
 */
void factory_test_send_response(const factory_test_frame_t* response) {
    uint8_t tx_buffer[FACTORY_TEST_BUFFER_SIZE];
    uint16_t tx_index = 0;

    // SOF (Little Endian)
    tx_buffer[tx_index++] = response->sof & 0xFF;
    tx_buffer[tx_index++] = (response->sof >> 8) & 0xFF;

    // Message info
    tx_buffer[tx_index++] = response->source;
    tx_buffer[tx_index++] = response->target;
    tx_buffer[tx_index++] = response->msg_id;

    // Payload length (Little Endian)
    tx_buffer[tx_index++] = response->payload_len & 0xFF;
    tx_buffer[tx_index++] = (response->payload_len >> 8) & 0xFF;

    // Payload
    memcpy(&tx_buffer[tx_index], response->payload, response->payload_len);
    tx_index += response->payload_len;

    // Calculate and add CRC16
    uint16_t crc = factory_test_calculate_crc16(&tx_buffer[2],
                                                3 + 2 + response->payload_len);
    tx_buffer[tx_index++] = crc & 0xFF;
    tx_buffer[tx_index++] = (crc >> 8) & 0xFF;

    // EOF (Little Endian)
    tx_buffer[tx_index++] = response->eof & 0xFF;
    tx_buffer[tx_index++] = (response->eof >> 8) & 0xFF;

    // Send via UART
    factory_test_send_data(tx_buffer, tx_index);

    elog_d(TAG, "Sent response frame, MSG_ID=0x%02X, len=%d", response->msg_id,
           tx_index);
}

/**
 * @brief  Handle heartbeat message
 * @param  frame: Pointer to received frame
 * @retval None
 */
void factory_test_handle_heartbeat(const factory_test_frame_t* frame) {
    factory_test_frame_t response;
    uint8_t status = HEARTBEAT_OK;

    factory_test_create_response_frame(&response, MSG_ID_HEARTBEAT, &status, 1);
    factory_test_send_response(&response);

    elog_d(TAG, "Heartbeat response sent");
}

/**
 * @brief  Handle GPIO control message (MSG_ID = 0x10)
 * @param  frame: Pointer to received frame
 * @retval None
 */
void factory_test_handle_gpio_control(const factory_test_frame_t* frame) {
    if (frame->payload_len < 2) {
        elog_w(TAG, "GPIO control payload too short");
        return;
    }

    gpio_control_cmd_t* cmd = (gpio_control_cmd_t*)frame->payload;
    factory_test_frame_t response;
    device_status_t status = DEVICE_OK;

    switch (cmd->sub_id) {
        case GPIO_SUB_SET_MODE:
            if (frame->payload_len >= 5) {
                status = factory_test_gpio_set_mode(cmd->port_id, cmd->pin_mask,
                                                    cmd->value);
                uint8_t response_payload[2] = {cmd->sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {cmd->sub_id,
                                               DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            }
            break;

        case GPIO_SUB_SET_PULL:
            if (frame->payload_len >= 5) {
                status = factory_test_gpio_set_pull(cmd->port_id, cmd->pin_mask,
                                                    cmd->value);
                uint8_t response_payload[2] = {cmd->sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {cmd->sub_id,
                                               DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            }
            break;

        case GPIO_SUB_WRITE_LEVEL:
            if (frame->payload_len >= 5) {
                status = factory_test_gpio_write_level(
                    cmd->port_id, cmd->pin_mask, cmd->value);
                uint8_t response_payload[2] = {cmd->sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {cmd->sub_id,
                                               DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_GPIO_CONTROL, response_payload, 2);
            }
            break;

        case GPIO_SUB_READ_LEVEL: {
            uint16_t levels = 0;
            status = factory_test_gpio_read_level(cmd->port_id, &levels);
            gpio_read_response_t read_response = {.sub_id = GPIO_SUB_READ_LEVEL,
                                                  .port_id = cmd->port_id,
                                                  .levels = levels};
            factory_test_create_response_frame(&response, MSG_ID_GPIO_CONTROL,
                                               (uint8_t*)&read_response,
                                               sizeof(read_response));
        } break;

        default: {
            uint8_t response_payload[2] = {cmd->sub_id,
                                           DEVICE_ERR_INVALID_SUB_ID};
            factory_test_create_response_frame(&response, MSG_ID_GPIO_CONTROL,
                                               response_payload, 2);
        } break;
    }

    factory_test_send_response(&response);
    elog_d(TAG, "GPIO control response sent, sub_id=0x%02X, status=%d",
           cmd->sub_id, status);
}

/**
 * @brief  Handle 64-way IO control message (MSG_ID = 0x11)
 * @param  frame: Pointer to received frame
 * @retval None
 */
void factory_test_handle_64way_io_control(const factory_test_frame_t* frame) {
    if (frame->payload_len < 1) {
        elog_w(TAG, "64-way IO control payload too short");
        return;
    }

    uint8_t sub_id = frame->payload[0];
    factory_test_frame_t response;
    device_status_t status = DEVICE_OK;

    elog_d(TAG, "64-way IO control: sub_id=0x%02X", sub_id);

    switch (sub_id) {
        case 0x01:    // Set mode
            if (frame->payload_len >=
                10) {    // Sub-ID(1) + Pin-Mask(8) + Value(1) = 10
                uint64_t pin_mask = 0;
                // 64-way IO - 8 bytes mask (little endian)
                for (int i = 0; i < 8; i++) {
                    pin_mask |= ((uint64_t)frame->payload[1 + i]) << (i * 8);
                }
                uint8_t value = frame->payload[9];

                status = io64_set_mode(pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            }
            break;

        case 0x02:    // Set pull
            if (frame->payload_len >= 10) {
                uint64_t pin_mask = 0;
                for (int i = 0; i < 8; i++) {
                    pin_mask |= ((uint64_t)frame->payload[1 + i]) << (i * 8);
                }
                uint8_t value = frame->payload[9];

                status = io64_set_pull(pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            }
            break;

        case 0x03:    // Write level
            if (frame->payload_len >= 10) {
                uint64_t pin_mask = 0;
                for (int i = 0; i < 8; i++) {
                    pin_mask |= ((uint64_t)frame->payload[1 + i]) << (i * 8);
                }
                uint8_t value = frame->payload[9];

                status = io64_write_level(pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            }
            break;

        case 0x04:    // Read level
        {
            uint64_t levels = 0;
            status = io64_read_level(&levels);

            if (status == DEVICE_OK) {
                // 64-way IO response: Sub-ID(1) + Status(1) + Levels(8)
                uint8_t response_payload[10];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                for (int i = 0; i < 8; i++) {
                    response_payload[2 + i] = (levels >> (i * 8)) & 0xFF;
                }
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 10);
            } else {
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
            }
        } break;

        default: {
            uint8_t response_payload[2] = {sub_id, DEVICE_ERR_INVALID_SUB_ID};
            factory_test_create_response_frame(
                &response, MSG_ID_64WAY_IO_CONTROL, response_payload, 2);
        } break;
    }

    factory_test_send_response(&response);
    elog_d(TAG, "64-way IO control response sent, sub_id=0x%02X, status=%d",
           sub_id, status);
}

/**
 * @brief  Handle DIP switch control message (MSG_ID = 0x12)
 * @param  frame: Pointer to received frame
 * @retval None
 */
void factory_test_handle_dip_switch_control(const factory_test_frame_t* frame) {
    if (frame->payload_len < 1) {
        elog_w(TAG, "DIP switch control payload too short");
        return;
    }

    uint8_t sub_id = frame->payload[0];
    factory_test_frame_t response;
    device_status_t status = DEVICE_OK;

    elog_d(TAG, "DIP switch control: sub_id=0x%02X", sub_id);

    switch (sub_id) {
        case 0x04:    // Read level
        {
            uint8_t levels = 0;
            status = dip_read_level(&levels);

            if (status == DEVICE_OK) {
                // DIP switch response: Sub-ID(1) + Status(1) + Levels(1)
                uint8_t response_payload[3];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                response_payload[2] = levels;
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 3);
            } else {
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            }
        } break;

        default: {
            uint8_t response_payload[2] = {sub_id, DEVICE_ERR_INVALID_SUB_ID};
            factory_test_create_response_frame(
                &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
        } break;
    }

    factory_test_send_response(&response);
    elog_d(TAG, "DIP switch control response sent, sub_id=0x%02X, status=%d",
           sub_id, status);
}

/**
 * @brief  Handle execute test message (MSG_ID = 0x30)
 * @param  frame: Pointer to received frame
 * @retval None
 */
void factory_test_handle_additional_test(const factory_test_frame_t* frame) {
    if (frame->payload_len < 1) {
        elog_w(TAG, "Additional test payload too short");
        return;
    }

    uint8_t sub_id = frame->payload[0];
    factory_test_frame_t response;
    device_status_t status = DEVICE_OK;

    elog_d(TAG, "Additional test: sub_id=0x%02X", sub_id);

    switch (sub_id) {
        case TEST_SUB_UART_LOOPBACK: {
            // Sub-ID 0x01: UART loopback test
            status = factory_test_uart_loopback();
            uint8_t response_payload[2] = {sub_id, status};
            factory_test_create_response_frame(
                &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
        } break;

        case TEST_SUB_WRITE_SN: {
            // Sub-ID 0x02: Write SN code
            if (frame->payload_len <
                7) {    // Sub-ID(1) + batch_id(4) + SN_length(1) + SN_data(>=1)
                status = DEVICE_ERR_EXECUTION;
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
            } else {
                // 提取batch_id (Little Endian) - 紧跟在sub_id后面
                uint32_t batch_id = frame->payload[1] |
                                   (frame->payload[2] << 8) |
                                   (frame->payload[3] << 16) |
                                   (frame->payload[4] << 24);
                
                uint8_t sn_length = frame->payload[5];
                const uint8_t* sn_data = &frame->payload[6];
                
                // 检查payload长度是否正确：Sub-ID(1) + batch_id(4) + SN_length(1) + SN_data(N)
                if (frame->payload_len != (6 + sn_length)) {
                    status = DEVICE_ERR_EXECUTION;
                } else {
                    status = factory_test_write_sn(sn_data, sn_length, batch_id);
                }

                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
            }
        } break;

        case TEST_SUB_READ_UID: {
            // Sub-ID 0x03: Read device UID
            uint8_t uid_data[DEVICE_UID_SIZE];
            status = factory_test_read_device_uid(uid_data);

            if (status == DEVICE_OK) {
                // Response: Sub-ID(1) + Status(1) + ID_length(1) + ID_data(12)
                uint8_t response_payload[15];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                response_payload[2] = DEVICE_UID_SIZE;
                memcpy(&response_payload[3], uid_data, DEVICE_UID_SIZE);
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 15);
            } else {
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
            }
        } break;

        case TEST_SUB_READ_SN: {
            // Sub-ID 0x04: Read SN code
            uint8_t sn_data[SN_MAX_LENGTH];
            uint8_t sn_length = 0;
            uint32_t batch_id = 0;
            status = factory_test_read_sn(sn_data, &sn_length, &batch_id);

            if (status == DEVICE_OK) {
                // Response: Sub-ID(1) + Status(1) + batch_id(4) + SN_length(1) + SN_data(N)
                uint8_t response_payload[7 + SN_MAX_LENGTH];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                // 添加batch_id (Little Endian) - 紧跟在status后面
                response_payload[2] = batch_id & 0xFF;
                response_payload[3] = (batch_id >> 8) & 0xFF;
                response_payload[4] = (batch_id >> 16) & 0xFF;
                response_payload[5] = (batch_id >> 24) & 0xFF;
                response_payload[6] = sn_length;
                memcpy(&response_payload[7], sn_data, sn_length);
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload,
                    7 + sn_length);
            } else {
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
            }
        } break;

        case TEST_SUB_READ_FW_VERSION: {
            // Sub-ID 0x05: Read firmware version
            uint8_t version_data[3];
            status = factory_test_read_firmware_version(version_data);

            if (status == DEVICE_OK) {
                // Response: Sub-ID(1) + Status(1) + FW_length(1) + FW_data(3)
                uint8_t response_payload[6];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                response_payload[2] = 3;                  // FW version length
                response_payload[3] = version_data[0];    // Major
                response_payload[4] = version_data[1];    // Minor
                response_payload[5] = version_data[2];    // Patch
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 6);
            } else {
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
            }
        } break;

        default: {
            elog_w(TAG, "Unknown additional test sub-ID: 0x%02X", sub_id);
            uint8_t response_payload[2] = {sub_id, DEVICE_ERR_INVALID_SUB_ID};
            factory_test_create_response_frame(
                &response, MSG_ID_ADDITIONAL_TEST, response_payload, 2);
        } break;
    }

    factory_test_send_response(&response);
    elog_d(TAG, "Additional test response sent, sub_id=0x%02X, status=%d",
           sub_id, status);
}

/* GPIO Control Functions --------------------------------------------------- */

/**
 * @brief  Set GPIO pin mode
 * @param  port_id: Port ID (0=PA, 1=PB, etc.)
 * @param  pin_mask: Pin mask (bit 0 = pin 0, etc.)
 * @param  mode: GPIO mode (0=input, 1=output, 2=analog)
 * @retval Device status
 */
device_status_t factory_test_gpio_set_mode(uint8_t port_id, uint16_t pin_mask,
                                           uint8_t mode) {
    GPIO_TypeDef* gpio_port = factory_test_get_gpio_port(port_id);
    if (gpio_port == NULL) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    // Enable GPIO clock for the specific port
    switch (port_id) {
        case PORT_ID_A:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case PORT_ID_B:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case PORT_ID_C:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case PORT_ID_D:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case PORT_ID_E:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case PORT_ID_F:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case PORT_ID_G:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case PORT_ID_H:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
        default:
            return DEVICE_ERR_INVALID_PORT;
    }

    uint32_t gpio_mode;

    // Set mode
    switch (mode) {
        case FACTORY_GPIO_MODE_INPUT:
            gpio_mode = GPIO_MODE_INPUT;
            break;
        case FACTORY_GPIO_MODE_OUTPUT:
            gpio_mode = GPIO_MODE_OUTPUT_PP;
            break;
        case FACTORY_GPIO_MODE_ANALOG:
            gpio_mode = GPIO_MODE_ANALOG;
            break;
        default:
            return DEVICE_ERR_EXECUTION;
    }

    // Configure each pin individually to preserve its own pull configuration
    for (uint8_t pin_num = 0; pin_num < 16; pin_num++) {
        if (pin_mask & (1 << pin_num)) {
            uint16_t current_pin = (1 << pin_num);

            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = current_pin;
            gpio_init.Mode = gpio_mode;
            // Preserve existing pull configuration for this specific pin
            gpio_init.Pull = get_gpio_pin_pull(gpio_port, current_pin);
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(gpio_port, &gpio_init);
        }
    }

    elog_d(TAG, "GPIO mode set: port=%d, mask=0x%04X, mode=%d", port_id,
           pin_mask, mode);
    return DEVICE_OK;
}

/**
 * @brief  Set GPIO pin pull configuration
 * @param  port_id: Port ID
 * @param  pin_mask: Pin mask
 * @param  pull: Pull configuration (0=down, 1=up, 2=none)
 * @retval Device status
 */
device_status_t factory_test_gpio_set_pull(uint8_t port_id, uint16_t pin_mask,
                                           uint8_t pull) {
    GPIO_TypeDef* gpio_port = factory_test_get_gpio_port(port_id);
    if (gpio_port == NULL) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    // Enable GPIO clock for the specific port
    switch (port_id) {
        case PORT_ID_A:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case PORT_ID_B:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case PORT_ID_C:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case PORT_ID_D:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case PORT_ID_E:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case PORT_ID_F:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case PORT_ID_G:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case PORT_ID_H:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
        default:
            return DEVICE_ERR_INVALID_PORT;
    }

    uint32_t gpio_pull;

    switch (pull) {
        case GPIO_PULL_DOWN:
            gpio_pull = GPIO_PULLDOWN;
            break;
        case GPIO_PULL_UP:
            gpio_pull = GPIO_PULLUP;
            break;
        case GPIO_PULL_NONE:
            gpio_pull = GPIO_NOPULL;
            break;
        default:
            return DEVICE_ERR_EXECUTION;
    }

    // Configure each pin individually to preserve its own mode configuration
    for (uint8_t pin_num = 0; pin_num < 16; pin_num++) {
        if (pin_mask & (1 << pin_num)) {
            uint16_t current_pin = (1 << pin_num);

            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = current_pin;
            // Preserve existing mode configuration for this specific pin
            gpio_init.Mode = get_gpio_pin_mode(gpio_port, current_pin);
            gpio_init.Pull = gpio_pull;
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(gpio_port, &gpio_init);
        }
    }

    elog_d(TAG, "GPIO pull set: port=%d, mask=0x%04X, pull=%d", port_id,
           pin_mask, pull);
    return DEVICE_OK;
}

/**
 * @brief  Write GPIO pin levels
 * @param  port_id: Port ID
 * @param  pin_mask: Pin mask
 * @param  level: Level to write (0=low, 1=high)
 * @retval Device status
 */
device_status_t factory_test_gpio_write_level(uint8_t port_id,
                                              uint16_t pin_mask,
                                              uint8_t level) {
    GPIO_TypeDef* gpio_port = factory_test_get_gpio_port(port_id);
    if (gpio_port == NULL) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    uint16_t hal_pin_mask = factory_test_convert_pin_mask(pin_mask);

    if (level == GPIO_LEVEL_HIGH) {
        HAL_GPIO_WritePin(gpio_port, hal_pin_mask, GPIO_PIN_SET);
    } else if (level == GPIO_LEVEL_LOW) {
        HAL_GPIO_WritePin(gpio_port, hal_pin_mask, GPIO_PIN_RESET);
    } else {
        return DEVICE_ERR_EXECUTION;
    }

    elog_d(TAG, "GPIO level written: port=%d, mask=0x%04X, level=%d", port_id,
           pin_mask, level);
    return DEVICE_OK;
}

/**
 * @brief  Read GPIO pin levels
 * @param  port_id: Port ID
 * @param  levels: Pointer to store read levels
 * @retval Device status
 */
device_status_t factory_test_gpio_read_level(uint8_t port_id,
                                             uint16_t* levels) {
    GPIO_TypeDef* gpio_port = factory_test_get_gpio_port(port_id);
    if (gpio_port == NULL) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (levels == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    *levels = (uint16_t)gpio_port->IDR;

    elog_d(TAG, "GPIO levels read: port=%d, levels=0x%04X", port_id, *levels);
    return DEVICE_OK;
}

/**
 * @brief  Get GPIO port from port ID
 * @param  port_id: Port ID
 * @retval GPIO port pointer or NULL if invalid
 */
GPIO_TypeDef* factory_test_get_gpio_port(uint8_t port_id) {
    switch (port_id) {
        case PORT_ID_A:
            return GPIOA;
        case PORT_ID_B:
            return GPIOB;
        case PORT_ID_C:
            return GPIOC;
        case PORT_ID_D:
            return GPIOD;
        case PORT_ID_E:
            return GPIOE;
        case PORT_ID_F:
            return GPIOF;
        case PORT_ID_G:
            return GPIOG;
        case PORT_ID_H:
            return GPIOH;
        default:
            return NULL;
    }
}

/**
 * @brief  Convert pin mask to HAL pin mask
 * @param  mask: Input pin mask (bit position format)
 * @retval HAL pin mask
 */
uint16_t factory_test_convert_pin_mask(uint16_t mask) {
    uint16_t hal_mask = 0;

    for (uint8_t i = 0; i < 16; i++) {
        if (mask & (1 << i)) {
            hal_mask |= (1 << i);
        }
    }

    return hal_mask;
}

/* Private functions ---------------------------------------------------------
 */

/**
 * @brief  Reset frame buffer
 * @retval None
 */
static void factory_test_reset_frame_buffer(void) {
    memset(frame_buffer, 0, sizeof(frame_buffer));
    frame_index = 0;
}

/**
 * @brief  Find frame start in buffer
 * @retval true if frame start found, false otherwise
 */
static bool factory_test_find_frame_start(void) {
    if (frame_index < 2) {
        return false;
    }

    // Look for SOF pattern (55 AA in little endian)
    for (uint16_t i = 0; i <= frame_index - 2; i++) {
        if (frame_buffer[i] == 0x55 && frame_buffer[i + 1] == 0xAA) {
            if (i > 0) {
                // Move frame to beginning of buffer
                memmove(frame_buffer, &frame_buffer[i], frame_index - i);
                frame_index -= i;
            }
            return true;
        }
    }

    // No SOF found, keep only the last byte in case it's the start of SOF
    if (frame_index > 1) {
        frame_buffer[0] = frame_buffer[frame_index - 1];
        frame_index = 1;
    }

    return false;
}

/**
 * @brief  Get expected frame length from buffer
 * @param  buffer: Buffer containing frame start
 * @retval Expected frame length or 0 if cannot determine
 */
static uint16_t factory_test_get_frame_length(const uint8_t* buffer) {
    if (frame_index < 7) {    // Need at least SOF(2) + MSG(3) + LEN(2)
        return 0;
    }

    // Extract payload length (Little Endian)
    uint16_t payload_len = buffer[FRAME_PAYLOAD_LEN_OFFSET] |
                           (buffer[FRAME_PAYLOAD_LEN_OFFSET + 1] << 8);

    if (payload_len > FACTORY_TEST_MAX_PAYLOAD) {
        return 0;
    }

    // Total frame length: SOF(2) + MSG(3) + LEN(2) + PAYLOAD + CRC(2) + EOF(2)
    return 11 + payload_len;
}

/**
 * @brief  Create response frame
 * @param  response: Pointer to response frame structure
 * @param  msg_id: Message ID
 * @param  payload: Payload data
 * @param  payload_len: Payload length
 * @retval None
 */
static void factory_test_create_response_frame(factory_test_frame_t* response,
                                               uint8_t msg_id,
                                               const uint8_t* payload,
                                               uint16_t payload_len) {
    response->sof = FACTORY_TEST_SOF;
    response->source = DEVICE_ADDR_BOARD;
    response->target = DEVICE_ADDR_TOOLING;
    response->msg_id = msg_id;
    response->payload_len = payload_len;

    if (payload && payload_len > 0) {
        memcpy(response->payload, payload, payload_len);
    }

    response->eof = FACTORY_TEST_EOF;
}

/* Ring buffer functions ---------------------------------------------------- */

/**
 * @brief  Put data into ring buffer (interrupt-safe)
 * @param  data: Data byte to store
 * @retval true if successful, false if buffer full
 */
static bool ring_buffer_put(uint8_t data) {
    uint16_t next_head = (ring_head + 1) % RING_BUFFER_SIZE;

    if (next_head == ring_tail) {
        // Buffer full
        return false;
    }

    ring_buffer[ring_head] = data;
    ring_head = next_head;
    return true;
}

/**
 * @brief  Get data from ring buffer
 * @param  data: Pointer to store retrieved data
 * @retval true if data available, false if buffer empty
 */
static bool ring_buffer_get(uint8_t* data) {
    if (ring_head == ring_tail) {
        // Buffer empty
        return false;
    }

    *data = ring_buffer[ring_tail];
    ring_tail = (ring_tail + 1) % RING_BUFFER_SIZE;
    return true;
}

/**
 * @brief  Get number of bytes available in ring buffer
 * @retval Number of bytes available
 */
__attribute__((unused)) static uint16_t ring_buffer_available(void) {
    return (ring_head + RING_BUFFER_SIZE - ring_tail) % RING_BUFFER_SIZE;
}

/**
 * @brief  Reset ring buffer
 * @retval None
 */
static void ring_buffer_reset(void) {
    ring_head = 0;
    ring_tail = 0;
}

/* GPIO configuration helper functions implementation ----------------------- */

/**
 * @brief  Get current GPIO pin mode
 * @param  port: GPIO port
 * @param  pin: GPIO pin (HAL format, e.g., GPIO_PIN_0)
 * @retval Current GPIO mode
 */
static uint32_t get_gpio_pin_mode(GPIO_TypeDef* port, uint16_t pin) {
    // Find pin position (0-15)
    uint8_t pin_pos = 0;
    uint16_t temp_pin = pin;
    while (temp_pin > 1) {
        temp_pin >>= 1;
        pin_pos++;
    }

    // Read mode from MODER register (2 bits per pin)
    uint32_t mode_bits = (port->MODER >> (pin_pos * 2)) & 0x03;

    switch (mode_bits) {
        case 0x00:
            return GPIO_MODE_INPUT;
        case 0x01:
            return GPIO_MODE_OUTPUT_PP;
        case 0x02:
            return GPIO_MODE_AF_PP;
        case 0x03:
            return GPIO_MODE_ANALOG;
        default:
            return GPIO_MODE_INPUT;
    }
}

/**
 * @brief  Get current GPIO pin pull configuration
 * @param  port: GPIO port
 * @param  pin: GPIO pin (HAL format, e.g., GPIO_PIN_0)
 * @retval Current GPIO pull configuration
 */
static uint32_t get_gpio_pin_pull(GPIO_TypeDef* port, uint16_t pin) {
    // Find pin position (0-15)
    uint8_t pin_pos = 0;
    uint16_t temp_pin = pin;
    while (temp_pin > 1) {
        temp_pin >>= 1;
        pin_pos++;
    }

    // Read pull from PUPDR register (2 bits per pin)
    uint32_t pull_bits = (port->PUPDR >> (pin_pos * 2)) & 0x03;

    switch (pull_bits) {
        case 0x00:
            return GPIO_NOPULL;
        case 0x01:
            return GPIO_PULLUP;
        case 0x02:
            return GPIO_PULLDOWN;
        default:
            return GPIO_NOPULL;
    }
}

/* 64-way IO control functions implementation ------------------------------- */

/**
 * @brief  Set 64-way IO pin mode
 * @param  pin_mask: Pin mask (64-bit)
 * @param  mode: GPIO mode
 * @retval Device status
 */
static device_status_t io64_set_mode(uint64_t pin_mask, uint8_t mode) {
    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    uint32_t gpio_mode;
    switch (mode) {
        case FACTORY_GPIO_MODE_INPUT:
            gpio_mode = GPIO_MODE_INPUT;
            break;
        case FACTORY_GPIO_MODE_OUTPUT:
            gpio_mode = GPIO_MODE_OUTPUT_PP;
            break;
        case FACTORY_GPIO_MODE_ANALOG:
            gpio_mode = GPIO_MODE_ANALOG;
            break;
        default:
            return DEVICE_ERR_EXECUTION;
    }

    // Configure each selected pin
    for (uint8_t i = 0; i < 64; i++) {
        if (pin_mask & (1ULL << i)) {
            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = io_pin_map[i].pin;
            gpio_init.Mode = gpio_mode;
            // Preserve existing pull configuration
            gpio_init.Pull =
                get_gpio_pin_pull(io_pin_map[i].port, io_pin_map[i].pin);
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(io_pin_map[i].port, &gpio_init);
        }
    }

    elog_d(TAG, "64-way IO mode set: mask=0x%016llX, mode=%d", pin_mask, mode);
    return DEVICE_OK;
}

/**
 * @brief  Set 64-way IO pin pull configuration
 * @param  pin_mask: Pin mask (64-bit)
 * @param  pull: Pull configuration
 * @retval Device status
 */
static device_status_t io64_set_pull(uint64_t pin_mask, uint8_t pull) {
    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    uint32_t gpio_pull;
    switch (pull) {
        case GPIO_PULL_DOWN:
            gpio_pull = GPIO_PULLDOWN;
            break;
        case GPIO_PULL_UP:
            gpio_pull = GPIO_PULLUP;
            break;
        case GPIO_PULL_NONE:
            gpio_pull = GPIO_NOPULL;
            break;
        default:
            return DEVICE_ERR_EXECUTION;
    }

    // Configure each selected pin
    for (uint8_t i = 0; i < 64; i++) {
        if (pin_mask & (1ULL << i)) {
            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = io_pin_map[i].pin;
            // Preserve existing mode configuration
            gpio_init.Mode =
                get_gpio_pin_mode(io_pin_map[i].port, io_pin_map[i].pin);
            gpio_init.Pull = gpio_pull;
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(io_pin_map[i].port, &gpio_init);
        }
    }

    elog_d(TAG, "64-way IO pull set: mask=0x%016llX, pull=%d", pin_mask, pull);
    return DEVICE_OK;
}

/**
 * @brief  Write 64-way IO pin levels
 * @param  pin_mask: Pin mask (64-bit)
 * @param  level: Level to write
 * @retval Device status
 */
static device_status_t io64_write_level(uint64_t pin_mask, uint8_t level) {
    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    GPIO_PinState pin_state;
    if (level == GPIO_LEVEL_HIGH) {
        pin_state = GPIO_PIN_SET;
    } else if (level == GPIO_LEVEL_LOW) {
        pin_state = GPIO_PIN_RESET;
    } else {
        return DEVICE_ERR_EXECUTION;
    }

    // Write level to each selected pin
    for (uint8_t i = 0; i < 64; i++) {
        if (pin_mask & (1ULL << i)) {
            HAL_GPIO_WritePin(io_pin_map[i].port, io_pin_map[i].pin, pin_state);
        }
    }

    elog_d(TAG, "64-way IO level written: mask=0x%016llX, level=%d", pin_mask,
           level);
    return DEVICE_OK;
}

/**
 * @brief  Read 64-way IO pin levels
 * @param  levels: Pointer to store read levels (64-bit)
 * @retval Device status
 */
static device_status_t io64_read_level(uint64_t* levels) {
    if (levels == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    *levels = 0;

    // Read level from each pin
    for (uint8_t i = 0; i < 64; i++) {
        GPIO_PinState pin_state =
            HAL_GPIO_ReadPin(io_pin_map[i].port, io_pin_map[i].pin);
        if (pin_state == GPIO_PIN_SET) {
            *levels |= (1ULL << i);
        }
    }

    elog_d(TAG, "64-way IO levels read: levels=0x%016llX", *levels);
    return DEVICE_OK;
}

/* SN management functions implementation ----------------------------------- */

/**
 * @brief  Erase Flash sector for SN storage
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
static HAL_StatusTypeDef flash_erase_sn_sector(void) {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    HAL_StatusTypeDef status;

    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    // 配置擦除参数
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;    // 2.7V-3.6V
    erase_init.Sector = SN_FLASH_SECTOR;
    erase_init.NbSectors = 1;

    // 执行擦除
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    // 锁定Flash
    HAL_FLASH_Lock();

    return status;
}

/**
 * @brief  Write SN data to Flash
 * @param  sn_storage: Pointer to SN storage structure
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
static HAL_StatusTypeDef flash_write_sn_data(const sn_storage_t* sn_storage) {
    HAL_StatusTypeDef status;
    uint32_t* src_ptr = (uint32_t*)sn_storage;
    uint32_t flash_addr = SN_FLASH_ADDRESS;
    uint32_t data_size = sizeof(sn_storage_t);
    uint32_t word_count = (data_size + 3) / 4;    // 向上取整到32位字

    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    // 按32位字写入数据
    for (uint32_t i = 0; i < word_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + (i * 4),
                                   src_ptr[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    // 锁定Flash
    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
 * @brief  Read SN data from Flash
 * @retval Pointer to SN storage structure in Flash
 */
static const sn_storage_t* get_sn_storage_ptr(void) {
    return (const sn_storage_t*)SN_FLASH_ADDRESS;
}

/**
 * @brief  Write SN code to Flash
 * @param  sn_data: Pointer to SN data
 * @param  sn_length: Length of SN data
 * @param  batch_id: Batch ID
 * @retval Device status
 */
device_status_t factory_test_write_sn(const uint8_t* sn_data,
                                      uint8_t sn_length, uint32_t batch_id) {
    if (sn_data == NULL || sn_length == 0 || sn_length > SN_MAX_LENGTH) {
        return DEVICE_ERR_EXECUTION;
    }

    sn_storage_t sn_storage = {0};

    // 准备SN存储结构
    sn_storage.magic = SN_MAGIC_NUMBER;
    sn_storage.batch_id = batch_id;
    sn_storage.sn_length = sn_length;
    memcpy(sn_storage.sn_data, sn_data, sn_length);

    // 擦除Flash扇区
    if (flash_erase_sn_sector() != HAL_OK) {
        elog_e(TAG, "Failed to erase SN Flash sector");
        return DEVICE_ERR_EXECUTION;
    }

    // 写入SN数据
    if (flash_write_sn_data(&sn_storage) != HAL_OK) {
        elog_e(TAG, "Failed to write SN data to Flash");
        return DEVICE_ERR_EXECUTION;
    }

    // 验证写入结果
    const sn_storage_t* stored_sn = get_sn_storage_ptr();
    if (stored_sn->magic != SN_MAGIC_NUMBER ||
        stored_sn->batch_id != batch_id ||
        stored_sn->sn_length != sn_length ||
        memcmp(stored_sn->sn_data, sn_data, sn_length) != 0) {
        elog_e(TAG, "SN data verification failed");
        return DEVICE_ERR_EXECUTION;
    }

    elog_i(TAG, "SN written successfully, length=%d, batch_id=0x%08X", sn_length, batch_id);
    return DEVICE_OK;
}

/**
 * @brief  Read SN code from Flash
 * @param  sn_data: Buffer to store SN data
 * @param  sn_length: Pointer to store SN length
 * @param  batch_id: Pointer to store batch ID
 * @retval Device status
 */
device_status_t factory_test_read_sn(uint8_t* sn_data, uint8_t* sn_length, uint32_t* batch_id) {
    if (sn_data == NULL || sn_length == NULL || batch_id == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    const sn_storage_t* stored_sn = get_sn_storage_ptr();

    // 检查魔数
    if (stored_sn->magic != SN_MAGIC_NUMBER) {
        elog_w(TAG, "SN magic number not found, SN not programmed");
        *sn_length = 0;
        *batch_id = 0;
        return DEVICE_ERR_EXECUTION;
    }

    // 检查SN长度有效性
    if (stored_sn->sn_length == 0 || stored_sn->sn_length > SN_MAX_LENGTH) {
        elog_w(TAG, "Invalid SN length: %d", stored_sn->sn_length);
        *sn_length = 0;
        *batch_id = 0;
        return DEVICE_ERR_EXECUTION;
    }

    // 复制SN数据和batch_id
    *sn_length = stored_sn->sn_length;
    *batch_id = stored_sn->batch_id;
    memcpy(sn_data, stored_sn->sn_data, stored_sn->sn_length);

    elog_d(TAG, "SN read successfully, length=%d, batch_id=0x%08X", *sn_length, *batch_id);
    return DEVICE_OK;
}

/* DIP switch control functions implementation ------------------------------ */

/**
 * @brief  Read DIP switch pin levels
 * @param  levels: Pointer to store read levels (8-bit)
 * @retval Device status
 */
static device_status_t dip_read_level(uint8_t* levels) {
    if (levels == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    *levels = 0;

    // Read level from each pin
    for (uint8_t i = 0; i < 8; i++) {
        GPIO_PinState pin_state =
            HAL_GPIO_ReadPin(dip_pin_map[i].port, dip_pin_map[i].pin);
        if (pin_state == GPIO_PIN_SET) {
            *levels |= (1 << i);
        }
    }

    elog_d(TAG, "DIP switch levels read: levels=0x%02X", *levels);
    return DEVICE_OK;
}

/* Additional test functions implementation --------------------------------- */

/**
 * @brief  Perform UART loopback test
 * @retval Device status
 */
device_status_t factory_test_uart_loopback(void) {
    uint8_t tx_buffer[] = UART_LOOPBACK_TEST_STRING;
    uint8_t rx_buffer[sizeof(UART_LOOPBACK_TEST_STRING)] = {0};
    uint32_t tx_len = strlen(UART_LOOPBACK_TEST_STRING);
    HAL_StatusTypeDef status;

    elog_d(TAG, "Starting UART loopback test");

    // // 清空接收缓冲区
    // memset(rx_buffer, 0, sizeof(rx_buffer));

    // // 发送测试字符串
    // status = HAL_UART_Transmit(&huart1, tx_buffer, tx_len,
    // UART_LOOPBACK_TIMEOUT); if (status != HAL_OK) {
    //     elog_e(TAG, "UART transmit failed: %d", status);
    //     return DEVICE_ERR_EXECUTION;
    // }

    // // 接收字符串
    // status = HAL_UART_Receive(&huart1, rx_buffer, tx_len,
    // UART_LOOPBACK_TIMEOUT); if (status != HAL_OK) {
    //     elog_e(TAG, "UART receive failed: %d", status);
    //     return DEVICE_ERR_EXECUTION;
    // }

    // // 比较发送和接收的数据
    // if (memcmp(tx_buffer, rx_buffer, tx_len) != 0) {
    //     elog_e(TAG, "UART loopback test failed - data mismatch");
    //     elog_e(TAG, "TX: %s", tx_buffer);
    //     elog_e(TAG, "RX: %s", rx_buffer);
    //     return DEVICE_ERR_EXECUTION;
    // }

    elog_i(TAG, "UART loopback test passed");
    return DEVICE_OK;
}

/**
 * @brief  Read device UID
 * @param  uid_data: Buffer to store UID data (12 bytes)
 * @retval Device status
 */
device_status_t factory_test_read_device_uid(uint8_t* uid_data) {
    if (uid_data == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    // 读取96位(12字节)设备UID
    uint32_t* uid_words = (uint32_t*)uid_data;

    // UID存储在3个32位寄存器中
    uid_words[0] = HAL_GetUIDw0();    // UID[31:0]
    uid_words[1] = HAL_GetUIDw1();    // UID[63:32]
    uid_words[2] = HAL_GetUIDw2();    // UID[95:64]

    elog_d(TAG, "Device UID: %08X%08X%08X", uid_words[2], uid_words[1],
           uid_words[0]);
    return DEVICE_OK;
}

/**
 * @brief  Read firmware version
 * @param  version_data: Buffer to store version data (3 bytes: major, minor,
 * patch)
 * @retval Device status
 */
device_status_t factory_test_read_firmware_version(uint8_t* version_data) {
    if (version_data == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    // 从编译时定义的宏中获取版本信息
    version_data[0] = (uint8_t)FIRMWARE_VERSION_MAJOR;
    version_data[1] = (uint8_t)FIRMWARE_VERSION_MINOR;
    version_data[2] = (uint8_t)FIRMWARE_VERSION_PATCH;

    elog_d(TAG, "Firmware version: %d.%d.%d", version_data[0], version_data[1],
           version_data[2]);
    return DEVICE_OK;
}
