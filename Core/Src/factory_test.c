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

#include "elog.h"
#include "main.h"
#include "usart.h"

/* Private variables ---------------------------------------------------------*/
static const char* TAG = "factory_test";
static factory_test_state_t test_state = FACTORY_TEST_DISABLED;

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

/* Special IO control functions */
static device_status_t special_io_set_mode(uint8_t target_id, uint64_t pin_mask,
                                           uint8_t mode);
static device_status_t special_io_set_pull(uint8_t target_id, uint64_t pin_mask,
                                           uint8_t pull);
static device_status_t special_io_write_level(uint8_t target_id,
                                              uint64_t pin_mask, uint8_t level);
static device_status_t special_io_read_level(uint8_t target_id,
                                             uint64_t* levels);
static void enable_gpio_clocks_for_special_io(uint8_t target_id);

/* GPIO configuration helper functions */
static uint32_t get_gpio_pin_mode(GPIO_TypeDef* port, uint16_t pin);
static uint32_t get_gpio_pin_pull(GPIO_TypeDef* port, uint16_t pin);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize factory test mode
 * @retval None
 */
void factory_test_init(void) {
    test_state = FACTORY_TEST_DISABLED;
    ring_buffer_reset();
    factory_test_reset_frame_buffer();

    elog_i(TAG, "Factory test initialized");
}

/**
 * @brief  Check if DIP6 pin is low to enter factory test mode
 * @retval true if should enter test mode, false otherwise
 */
bool factory_test_check_entry_condition(void) {
    // Check DIP6_Pin (PF9) state
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(DIP6_GPIO_Port, DIP6_Pin);

    if (pin_state == GPIO_PIN_RESET) {
        elog_i(TAG, "DIP6 pin detected low, entering factory test mode");
        return true;
    }

    return false;
}

/**
 * @brief  Enter factory test mode
 * @retval None
 */
void factory_test_enter_mode(void) {
    test_state = FACTORY_TEST_ENABLED;
    ring_buffer_reset();
    factory_test_reset_frame_buffer();
    elog_set_filter_tag_lvl(TAG, ELOG_LVL_ERROR);

    elog_i(TAG, "Entered factory test mode");

    // Send confirmation message
    const char* msg = "Factory test mode activated (DIP6 detected)\r\n";
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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
                        case MSG_ID_EXECUTE_TEST:
                            elog_v(TAG, "execute_test");
                            factory_test_handle_execute_test(&frame);
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
    HAL_UART_Transmit(&DEBUG_UART, tx_buffer, tx_index, HAL_MAX_DELAY);

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
            if (frame->payload_len >= 10) {    // Sub-ID(1) + Pin-Mask(8) + Value(1) = 10
                uint64_t pin_mask = 0;
                // 64-way IO - 8 bytes mask (little endian)
                for (int i = 0; i < 8; i++) {
                    pin_mask |= ((uint64_t)frame->payload[1 + i]) << (i * 8);
                }
                uint8_t value = frame->payload[9];

                status = special_io_set_mode(0x01, pin_mask, value);
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

                status = special_io_set_pull(0x01, pin_mask, value);
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

                status = special_io_write_level(0x01, pin_mask, value);
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
            status = special_io_read_level(0x01, &levels);

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
        case 0x01:    // Set mode
            if (frame->payload_len >= 3) {    // Sub-ID(1) + Pin-Mask(1) + Value(1) = 3
                uint8_t pin_mask = frame->payload[1];
                uint8_t value = frame->payload[2];

                status = special_io_set_mode(0x02, pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            }
            break;

        case 0x02:    // Set pull
            if (frame->payload_len >= 3) {
                uint8_t pin_mask = frame->payload[1];
                uint8_t value = frame->payload[2];

                status = special_io_set_pull(0x02, pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            }
            break;

        case 0x03:    // Write level
            if (frame->payload_len >= 3) {
                uint8_t pin_mask = frame->payload[1];
                uint8_t value = frame->payload[2];

                status = special_io_write_level(0x02, pin_mask, value);
                uint8_t response_payload[2] = {sub_id, status};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            } else {
                uint8_t response_payload[2] = {sub_id, DEVICE_ERR_EXECUTION};
                factory_test_create_response_frame(
                    &response, MSG_ID_DIP_SWITCH_CONTROL, response_payload, 2);
            }
            break;

        case 0x04:    // Read level
        {
            uint64_t levels = 0;
            status = special_io_read_level(0x02, &levels);

            if (status == DEVICE_OK) {
                // DIP switch response: Sub-ID(1) + Status(1) + Levels(1)
                uint8_t response_payload[3];
                response_payload[0] = sub_id;
                response_payload[1] = status;
                response_payload[2] = levels & 0xFF;
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
void factory_test_handle_execute_test(const factory_test_frame_t* frame) {
    // TODO: Implement test execution (UART loopback, read UID, etc.)
    elog_w(TAG, "Execute test not implemented yet");

    factory_test_frame_t response;
    uint8_t response_payload[2] = {0x01,
                                   DEVICE_ERR_EXECUTION};    // Sub-ID, Status
    factory_test_create_response_frame(&response, MSG_ID_EXECUTE_TEST,
                                       response_payload, 2);
    factory_test_send_response(&response);
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

/* Special IO control functions implementation -------------------------------
 */

/**
 * @brief  Enable GPIO clocks for special IO targets
 * @param  target_id: Target ID (1=64-way IO, 2=DIP switches)
 * @retval None
 */
static void enable_gpio_clocks_for_special_io(uint8_t target_id) {
    // already enabled in main.c
}

/**
 * @brief  Set special IO pin mode
 * @param  target_id: Target ID (1=64-way IO, 2=DIP switches)
 * @param  pin_mask: Pin mask
 * @param  mode: GPIO mode
 * @retval Device status
 */
static device_status_t special_io_set_mode(uint8_t target_id, uint64_t pin_mask,
                                           uint8_t mode) {
    if (target_id != 0x01 && target_id != 0x02) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    enable_gpio_clocks_for_special_io(target_id);

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

    const gpio_pin_map_t* pin_map;
    uint8_t pin_count;

    if (target_id == 0x01) {
        pin_map = io_pin_map;
        pin_count = 64;
    } else {
        pin_map = dip_pin_map;
        pin_count = 8;
    }

    // Configure each selected pin
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_mask & (1ULL << i)) {
            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = pin_map[i].pin;
            gpio_init.Mode = gpio_mode;
            // Preserve existing pull configuration
            gpio_init.Pull = get_gpio_pin_pull(pin_map[i].port, pin_map[i].pin);
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(pin_map[i].port, &gpio_init);
        }
    }

    elog_d(TAG, "Special IO mode set: target=%d, mask=0x%016X, mode=%d",
           target_id, pin_mask, mode);
    return DEVICE_OK;
}

/**
 * @brief  Set special IO pin pull configuration
 * @param  target_id: Target ID
 * @param  pin_mask: Pin mask
 * @param  pull: Pull configuration
 * @retval Device status
 */
static device_status_t special_io_set_pull(uint8_t target_id, uint64_t pin_mask,
                                           uint8_t pull) {
    if (target_id != 0x01 && target_id != 0x02) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (pin_mask == 0) {
        return DEVICE_ERR_INVALID_PIN;
    }

    enable_gpio_clocks_for_special_io(target_id);

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

    const gpio_pin_map_t* pin_map;
    uint8_t pin_count;

    if (target_id == 0x01) {
        pin_map = io_pin_map;
        pin_count = 64;
    } else {
        pin_map = dip_pin_map;
        pin_count = 8;
    }

    // Configure each selected pin
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_mask & (1ULL << i)) {
            GPIO_InitTypeDef gpio_init = {0};
            gpio_init.Pin = pin_map[i].pin;
            // Preserve existing mode configuration
            gpio_init.Mode = get_gpio_pin_mode(pin_map[i].port, pin_map[i].pin);
            gpio_init.Pull = gpio_pull;
            gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

            HAL_GPIO_Init(pin_map[i].port, &gpio_init);
        }
    }

    elog_d(TAG, "Special IO pull set: target=%d, mask=0x%016X, pull=%d",
           target_id, pin_mask, pull);
    return DEVICE_OK;
}

/**
 * @brief  Write special IO pin levels
 * @param  target_id: Target ID
 * @param  pin_mask: Pin mask
 * @param  level: Level to write
 * @retval Device status
 */
static device_status_t special_io_write_level(uint8_t target_id,
                                              uint64_t pin_mask,
                                              uint8_t level) {
    if (target_id != 0x01 && target_id != 0x02) {
        return DEVICE_ERR_INVALID_PORT;
    }

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

    const gpio_pin_map_t* pin_map;
    uint8_t pin_count;

    if (target_id == 0x01) {
        pin_map = io_pin_map;
        pin_count = 64;
    } else {
        pin_map = dip_pin_map;
        pin_count = 8;
    }

    // Write level to each selected pin
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_mask & (1ULL << i)) {
            HAL_GPIO_WritePin(pin_map[i].port, pin_map[i].pin, pin_state);
        }
    }

    elog_d(TAG, "Special IO level written: target=%d, mask=0x%016X, level=%d",
           target_id, pin_mask, level);
    return DEVICE_OK;
}

/**
 * @brief  Read special IO pin levels
 * @param  target_id: Target ID
 * @param  levels: Pointer to store read levels
 * @retval Device status
 */
static device_status_t special_io_read_level(uint8_t target_id,
                                             uint64_t* levels) {
    if (target_id != 0x01 && target_id != 0x02) {
        return DEVICE_ERR_INVALID_PORT;
    }

    if (levels == NULL) {
        return DEVICE_ERR_EXECUTION;
    }

    const gpio_pin_map_t* pin_map;
    uint8_t pin_count;

    if (target_id == 0x01) {
        pin_map = io_pin_map;
        pin_count = 64;
    } else {
        pin_map = dip_pin_map;
        pin_count = 8;
    }

    *levels = 0;

    // Read level from each pin
    for (uint8_t i = 0; i < pin_count; i++) {
        GPIO_PinState pin_state =
            HAL_GPIO_ReadPin(pin_map[i].port, pin_map[i].pin);
        if (pin_state == GPIO_PIN_SET) {
            *levels |= (1ULL << i);
        }
    }

    elog_d(TAG, "Special IO levels read: target=%d, levels=0x%016llX",
           target_id, *levels);
    return DEVICE_OK;
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
        case 0x00: return GPIO_MODE_INPUT;
        case 0x01: return GPIO_MODE_OUTPUT_PP;
        case 0x02: return GPIO_MODE_AF_PP;
        case 0x03: return GPIO_MODE_ANALOG;
        default: return GPIO_MODE_INPUT;
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
        case 0x00: return GPIO_NOPULL;
        case 0x01: return GPIO_PULLUP;
        case 0x02: return GPIO_PULLDOWN;
        default: return GPIO_NOPULL;
    }
}
