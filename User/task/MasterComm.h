#ifndef UWB_TASK_H
#define UWB_TASK_H

#include "cmsis_os2.h"
#include <stdint.h>

#define FRAME_LEN_MAX 1016
typedef enum
{
    UWB_MSG_TYPE_SEND_DATA = 1, // 应用任务发送数据
    UWB_MSG_TYPE_CONFIG,        // 配置信息
    UWB_MSG_TYPE_SET_MODE       // 设置工作模式
} UwbMsgType;

typedef struct
{
    UwbMsgType type;
    uint16_t dataLen;
    uint8_t data[FRAME_LEN_MAX];
    uint32_t delay_ms; // 发送延迟时间
} UwbTxMsg;

// UWB接收消息结构体
typedef struct
{
    uint16_t dataLen;
    uint8_t data[FRAME_LEN_MAX];
    uint32_t timestamp; // 接收时间戳
    uint32_t statusReg; // 状态寄存器值
} uwbRxMsg;

// 接收数据回调函数指针
typedef void (*UwbRxCallback)(const uwbRxMsg *msg);

class MasterComm
{
  public:
    MasterComm();
    ~MasterComm();

    int SendData(const uint8_t *data, uint16_t len, uint32_t delayMs);
    int ReceiveData(uwbRxMsg *msg, uint32_t timeoutMs);
    int Reconfigure();
    void SetRxCallback(UwbRxCallback callback);

  private:
    int Initialize(void);
    int GetTxQueueCount();
    int GetRxQueueCount();
    void ClearTxQueue();
    void ClearRxQueue();

    // 任务相关私有方法
    void UwbCommTask();
    static void UwbCommTaskWrapper(void *argument);

    // 私有成员变量
    osMessageQueueId_t uwbTxQueue; // UWB发送队列
    osMessageQueueId_t uwbRxQueue; // UWB接收队列
    osThreadId_t uwbCommTaskHandle;
    osSemaphoreId_t uwbTxSemaphore; // UWB发送信号量
    UwbRxCallback uwbRxCallback;    // 接收数据回调函数指针
};
#endif /* UWB_TASK_H */