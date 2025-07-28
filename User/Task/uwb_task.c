#include "cmsis_os2.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "elog.h"
#include "port.h"

#define FRAME_LEN_MAX 127
#define TX_QUEUE_SIZE 10
#define RX_QUEUE_SIZE 10

// UWB消息类型定义
typedef enum {
    UWB_MSG_TYPE_SEND_DATA = 1,    // 应用任务发送数据
    UWB_MSG_TYPE_CONFIG,           // 配置信息
    UWB_MSG_TYPE_SET_MODE          // 设置工作模式
} uwb_msg_type_t;

// UWB发送消息结构体
typedef struct {
    uwb_msg_type_t type;
    uint16_t data_len;
    uint8_t data[FRAME_LEN_MAX];
    uint32_t delay_ms;    // 发送延迟时间
} uwb_tx_msg_t;

// UWB接收消息结构体
typedef struct {
    uint16_t data_len;
    uint8_t data[FRAME_LEN_MAX];
    uint32_t timestamp;     // 接收时间戳
    uint32_t status_reg;    // 状态寄存器值
} uwb_rx_msg_t;

// 全局变量
static osMessageQueueId_t uwb_txQueue;    // UWB发送队列
static osMessageQueueId_t uwb_rxQueue;    // UWB接收队列
static osThreadId_t uwbCommTaskHandle;

// 接收数据回调函数指针
typedef void (*uwb_rx_callback_t)(const uwb_rx_msg_t *msg);
static uwb_rx_callback_t uwb_rx_callback = NULL;

/* Default communication configuration. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC
                        size). Used in RX only. */
};

static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint32_t status_reg = 0;
static uint16_t frame_len = 0;

// UWB通信任务
static void uwb_comm_task(void *argument) {
    static const char *TAG = "uwb_comm";
    uwb_tx_msg_t tx_msg;
    uwb_rx_msg_t rx_msg;

    // 初始化DW1000
    reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        elog_e(TAG, "dwt_initialise failed");
        osThreadExit();
    }
    port_set_dw1000_fastrate();

    // 配置DW1000
    dwt_configure(&config);
    elog_i(TAG, "dwt_configure success");

    uint32_t device_id = dwt_readdevid();
    elog_i(TAG, "device_id: %08X", device_id);

    // 启动接收模式
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1) {
        // 检查是否有发送消息
        if (osMessageQueueGet(uwb_txQueue, &tx_msg, NULL, 0) == osOK) {
            switch (tx_msg.type) {
                case UWB_MSG_TYPE_SEND_DATA:
                    dwt_forcetrxoff();    // 保证发送前DW1000已空闲

                    // 发送UWB数据
                    // DW1000会自动添加2字节CRC，所以实际写入的数据长度是用户数据长度
                    // 但是dwt_writetxfctrl需要包含CRC的总长度
                    dwt_writetxdata(tx_msg.data_len + 2, tx_msg.data, 0);
                    dwt_writetxfctrl(tx_msg.data_len + 2, 0, 0);
                    dwt_starttx(DWT_START_TX_IMMEDIATE);

                    // 等待发送完成
                    while (
                        !(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                        osDelay(1);
                    }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    // 发送完成后重新启动接收
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    // elog_i(TAG, "Sent %d bytes done", tx_msg.data_len);
                    break;

                case UWB_MSG_TYPE_CONFIG:
                    // 重新配置DW1000
                    dwt_configure(&config);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    elog_i(TAG, "Config updated");
                    break;

                case UWB_MSG_TYPE_SET_MODE:
                    // 设置工作模式（预留接口）
                    elog_i(TAG, "Mode set");
                    break;

                default:
                    break;
            }
        }

        // 检查是否有接收数据
        status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        if (status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) {
            elog_i(TAG, "status_reg: %08X", status_reg);
            if (status_reg & SYS_STATUS_RXFCG) {
                // 成功接收到数据
                frame_len =
                    dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                // elog_i(TAG, "frame_len: %d", frame_len);

                // frame_len包含2字节CRC，需要减去CRC长度得到实际数据长度
                if (frame_len >= 2 && frame_len <= FRAME_LEN_MAX) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);

                    // 构造接收消息，只包含用户数据，不包含CRC
                    rx_msg.data_len = frame_len - 2;    // 减去2字节CRC
                    for (int i = 0; i < rx_msg.data_len; i++) {
                        rx_msg.data[i] = rx_buffer[i];
                    }
                    rx_msg.timestamp = osKernelGetTickCount();
                    rx_msg.status_reg = status_reg;

                    // 将数据放入接收队列
                    osMessageQueuePut(uwb_rxQueue, &rx_msg, 0, 0);

                    // 如果有回调函数，调用它
                    if (uwb_rx_callback != NULL) {
                        uwb_rx_callback(&rx_msg);
                    }
                }

                // 清除接收完成标志
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            } else {
                // 接收错误
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                elog_e(TAG, "RX error: %08X", status_reg);
            }

            // 重新启动接收
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        osDelay(1);    // 防止任务占满CPU
    }
}

// 初始化UWB通信任务
void UWB_Task_Init(void) {
    static const char *TAG = "uwb_init";

    // 创建消息队列
    uwb_txQueue = osMessageQueueNew(TX_QUEUE_SIZE, sizeof(uwb_tx_msg_t), NULL);
    if (uwb_txQueue == NULL) {
        elog_e(TAG, "Failed to create UWB TX queue");
        return;
    }

    uwb_rxQueue = osMessageQueueNew(RX_QUEUE_SIZE, sizeof(uwb_rx_msg_t), NULL);
    if (uwb_rxQueue == NULL) {
        elog_e(TAG, "Failed to create UWB RX queue");
        return;
    }
    // 创建UWB通信任务
    const osThreadAttr_t uwbTask_attributes = {
        .name = "uwbCommTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    uwbCommTaskHandle = osThreadNew(uwb_comm_task, NULL, &uwbTask_attributes);

    if (uwbCommTaskHandle == NULL) {
        elog_e(TAG, "Failed to create UWB communication task");
    } else {
        elog_i(TAG, "UWB communication task initialized");
    }
}

// API函数：发送UWB数据
int UWB_SendData(const uint8_t *data, uint16_t len, uint32_t delay_ms) {
    if (data == NULL || len == 0 || len > FRAME_LEN_MAX) {
        return -1;
    }

    uwb_tx_msg_t msg;
    msg.type = UWB_MSG_TYPE_SEND_DATA;
    msg.data_len = len;
    msg.delay_ms = delay_ms;

    // 复制数据到消息结构体
    for (uint16_t i = 0; i < len; i++) {
        msg.data[i] = data[i];
    }

    // 发送到队列
    if (osMessageQueuePut(uwb_txQueue, &msg, 0, 100) != osOK) {
        return -3;    // 队列满或超时
    }

    return 0;    // 成功
}

// API函数：接收UWB数据（非阻塞）
int UWB_ReceiveData(uwb_rx_msg_t *msg, uint32_t timeout_ms) {
    if (msg == NULL) {
        return -1;
    }

    if (osMessageQueueGet(uwb_rxQueue, msg, NULL, timeout_ms) == osOK) {
        return 0;    // 成功
    }

    return -1;    // 超时或错误
}

// API函数：设置接收回调函数
void UWB_SetRxCallback(uwb_rx_callback_t callback) {
    uwb_rx_callback = callback;
}

// API函数：获取队列状态
int UWB_GetTxQueueCount(void) {
    return (int)osMessageQueueGetCount(uwb_txQueue);
}

int UWB_GetRxQueueCount(void) {
    return (int)osMessageQueueGetCount(uwb_rxQueue);
}

// API函数：清空队列
void UWB_ClearTxQueue(void) {
    uwb_tx_msg_t msg;
    while (osMessageQueueGet(uwb_txQueue, &msg, NULL, 0) == osOK) {
        // 清空队列
    }
}

void UWB_ClearRxQueue(void) {
    uwb_rx_msg_t msg;
    while (osMessageQueueGet(uwb_rxQueue, &msg, NULL, 0) == osOK) {
        // 清空队列
    }
}

// API函数：重新配置UWB
int UWB_Reconfigure(void) {
    uwb_tx_msg_t msg;
    msg.type = UWB_MSG_TYPE_CONFIG;
    msg.data_len = 0;

    if (osMessageQueuePut(uwb_txQueue, &msg, 0, 100) != osOK) {
        return -1;    // 队列满或超时
    }

    return 0;    // 成功
}