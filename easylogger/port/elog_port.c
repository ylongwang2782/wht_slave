/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */

#include <elog.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for SlaveDevice
typedef struct SlaveDevice SlaveDevice;

// Global pointer to SlaveDevice instance for synchronized timestamp
static void* g_slave_device_instance = NULL;

// Function pointer to get synchronized timestamp from SlaveDevice
static uint32_t (*get_sync_timestamp_ms_func)(void* device) = NULL;

#ifdef __cplusplus
}
#endif

extern osSemaphoreId_t elog_lockHandle;
extern osSemaphoreId_t elog_asyncHandle;
extern osSemaphoreId_t elog_dma_lockHandle;

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */

    return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
    RS485_TX_EN();
    HAL_UART_Transmit_DMA(&RS485_UART, (uint8_t *)log, size);
    osSemaphoreAcquire(elog_dma_lockHandle, osWaitForever);
    // RS485_RX_EN() 在DMA传输完成回调中调用
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    osSemaphoreAcquire(elog_lockHandle, osWaitForever);
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) { osSemaphoreRelease(elog_lockHandle); }

/**
 * Set SlaveDevice instance for synchronized timestamp
 *
 * @param device pointer to SlaveDevice instance
 * @param timestamp_func function to get synchronized timestamp in milliseconds
 */
void ElogSetSlaveDevice(void* device, uint32_t (*timestamp_func)(void*)) {
    g_slave_device_instance = device;
    get_sync_timestamp_ms_func = timestamp_func;
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    static char cur_system_time[16] = "";
    
    // Use synchronized timestamp if SlaveDevice is available
    if (g_slave_device_instance != NULL && get_sync_timestamp_ms_func != NULL) {
        uint32_t sync_timestamp_ms = get_sync_timestamp_ms_func(g_slave_device_instance);
        snprintf(cur_system_time, 16, "%lu", sync_timestamp_ms);
    } else {
        // Fallback to system tick count
        snprintf(cur_system_time, 16, "%lu", osKernelGetTickCount());
    }
    
    return cur_system_time;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) { return ""; }

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) { return ""; }

void elog_async_output_notice(void) { osSemaphoreRelease(elog_asyncHandle); }

void elog_entry(void *para) {
    size_t get_log_size = 0;
#ifdef ELOG_ASYNC_LINE_OUTPUT
    static char poll_get_buf[ELOG_LINE_BUF_SIZE - 4];
#else
    static char poll_get_buf[ELOG_ASYNC_OUTPUT_BUF_SIZE - 4];
#endif

    for (;;) {
        /* waiting log */
        osSemaphoreAcquire(elog_asyncHandle, osWaitForever);
        /* polling gets and outputs the log */
        while (1) {
#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size =
                elog_async_get_line_log(poll_get_buf, sizeof(poll_get_buf));
#else
            get_log_size =
                elog_async_get_log(poll_get_buf, sizeof(poll_get_buf));
#endif
            if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
            } else {
                break;
            }
        }
    }
}
