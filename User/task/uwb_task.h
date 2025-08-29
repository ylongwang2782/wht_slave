#ifndef UWB_TASK_H
#define UWB_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_LEN_MAX 1016

// UWB接收消息结构体
typedef struct {
    uint16_t data_len;
    uint8_t data[FRAME_LEN_MAX];
    uint32_t timestamp;     // 接收时间戳
    uint32_t status_reg;    // 状态寄存器值
} uwb_rx_msg_t;

// 接收数据回调函数指针
typedef void (*uwb_rx_callback_t)(const uwb_rx_msg_t *msg);

// 初始化UWB通信任务
void UWB_Task_Init(void);

// API函数：发送UWB数据
// 参数：data - 要发送的数据, len - 数据长度, delay_ms - 发送延迟时间（毫秒）
// 返回：0 - 成功, -1 - 参数错误, -3 - 队列满或超时
int UWB_SendData(const uint8_t *data, uint16_t len, uint32_t delay_ms);

// API函数：接收UWB数据（非阻塞）
// 参数：msg - 接收消息缓冲区, timeout_ms - 超时时间（毫秒）
// 返回：0 - 成功, -1 - 超时或错误
int UWB_ReceiveData(uwb_rx_msg_t *msg, uint32_t timeout_ms);

// API函数：设置接收回调函数
// 参数：callback - 回调函数指针，当接收到数据时自动调用
void UWB_SetRxCallback(uwb_rx_callback_t callback);

// API函数：获取队列状态
int UWB_GetTxQueueCount(void);    // 获取发送队列中的消息数量
int UWB_GetRxQueueCount(void);    // 获取接收队列中的消息数量

// API函数：清空队列
void UWB_ClearTxQueue(void);    // 清空发送队列
void UWB_ClearRxQueue(void);    // 清空接收队列

// API函数：重新配置UWB
// 返回：0 - 成功, -1 - 队列满或超时
int UWB_Reconfigure(void);

#ifdef __cplusplus
}
#endif

#endif /* UWB_TASK_H */