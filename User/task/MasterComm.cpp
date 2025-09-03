#include "MasterComm.h"

#include "cmsis_os2.h"

// #include "deca_device_api.h"
// #include "deca_regs.h"
// #include "port.h"
#include "CX310.hpp"
#include "uwb_interface.hpp"

// 外部声明全局指针
extern CX310_SlaveSpiAdapter *g_uwbAdapter;

#include "elog.h"

#define TX_QUEUE_SIZE 10
#define RX_QUEUE_SIZE 10

// 静态包装函数，用于FreeRTOS任务创建
void MasterComm::UwbCommTaskWrapper(void *argument)
{
    MasterComm *instance = static_cast<MasterComm *>(argument);
    instance->UwbCommTask();
}

// 实际的任务实现方法
void MasterComm::UwbCommTask()
{
    static auto TAG = "uwb_comm";
    UwbTxMsg txMsg;
    uwbRxMsg rxMsg;

    CX310<CX310_SlaveSpiAdapter> uwb;
    // 设置全局指针，用于中断处理
    g_uwbAdapter = &uwb.get_interface();

    std::vector<uint8_t> buffer = {0};

    if (uwb.init())
    {
        elog_i(TAG, "uwb.init success");
    }
    osDelay(3);
    uwb.set_recv_mode();

    for (;;)
    {
        // 等待发送信号量，确保队列中有完整的数据
        if (osSemaphoreAcquire(uwbTxSemaphore, 0) == osOK)
        {
            // 从队列获取发送消息
            if (osMessageQueueGet(uwbTxQueue, &txMsg, nullptr, 0) == osOK)
            {
                std::vector<uint8_t> tx_data(txMsg.data, txMsg.data + txMsg.dataLen);
                elog_i(TAG, "tx begin");
                uwb.update();
                uwb.data_transmit(tx_data);
            }
        }

        if (uwb.get_recv_data(buffer))
        {
            rxMsg.dataLen = buffer.size();
            for (int i = 0; i < rxMsg.dataLen; i++)
            {
                rxMsg.data[i] = buffer[i];
            }
            rxMsg.timestamp = osKernelGetTickCount();
            rxMsg.statusReg = 0;
            osMessageQueuePut(uwbRxQueue, &rxMsg, 0, 0);

            // 如果有回调函数，调用它
            if (uwbRxCallback != nullptr)
            {
                uwbRxCallback(&rxMsg);
            }
        }

        uwb.update();
        osDelay(1);
    }
}

MasterComm::MasterComm()
    : uwbTxQueue(nullptr), uwbRxQueue(nullptr), uwbCommTaskHandle(nullptr), uwbTxSemaphore(nullptr),
      uwbRxCallback(nullptr)
{
    Initialize();
}
MasterComm::~MasterComm()
{
    ClearTxQueue();
    ClearRxQueue();
}

int MasterComm::Initialize(void)
{
    static const char *TAG = "uwb_init";

    // 创建消息队列
    uwbTxQueue = osMessageQueueNew(TX_QUEUE_SIZE, sizeof(UwbTxMsg), nullptr);
    if (uwbTxQueue == nullptr)
    {
        elog_e(TAG, "Failed to create UWB TX queue");
        return -1;
    }

    uwbRxQueue = osMessageQueueNew(RX_QUEUE_SIZE, sizeof(uwbRxMsg), nullptr);
    if (uwbRxQueue == nullptr)
    {
        elog_e(TAG, "Failed to create UWB RX queue");
        return -1;
    }

    // 创建发送信号量（计数信号量，初始值为0）
    uwbTxSemaphore = osSemaphoreNew(TX_QUEUE_SIZE, 0, nullptr);
    if (uwbTxSemaphore == nullptr)
    {
        elog_e(TAG, "Failed to create UWB TX semaphore");
        return -1;
    }

    // 创建UWB通信任务
    constexpr osThreadAttr_t uwbTaskAttributes = {
        .name = "uwbCommTask",
        .stack_size = 512 * 16,
        .priority = (osPriority_t)osPriorityNormal,
    };

    uwbCommTaskHandle = osThreadNew(UwbCommTaskWrapper, this, &uwbTaskAttributes);
    return 0;
}

int MasterComm::SendData(const uint8_t *data, uint16_t len, uint32_t delayMs)
{
    if (data == nullptr || len == 0 || len > FRAME_LEN_MAX)
    {
        return -1;
    }

    UwbTxMsg msg;
    msg.type = UWB_MSG_TYPE_SEND_DATA;
    msg.dataLen = len;
    msg.delay_ms = delayMs;

    // 复制数据到消息结构体
    for (uint16_t i = 0; i < len; i++)
    {
        msg.data[i] = data[i];
    }

    // 发送到队列
    if (osMessageQueuePut(uwbTxQueue, &msg, 0, 100) != osOK)
    {
        return -3; // 队列满或超时
    }

    // 队列数据放入成功后，释放信号量通知通信任务
    osSemaphoreRelease(uwbTxSemaphore);

    return 0;
}

int MasterComm::ReceiveData(uwbRxMsg *msg, uint32_t timeoutMs)
{
    if (msg == nullptr)
    {
        return -1;
    }

    if (osMessageQueueGet(uwbRxQueue, msg, nullptr, timeoutMs) == osOK)
    {
        return 0; // 成功
    }

    return -1; // 超时或错误
}

void MasterComm::SetRxCallback(UwbRxCallback callback)
{
    this->uwbRxCallback = callback;
}

int MasterComm::GetTxQueueCount()
{
    return (int)osMessageQueueGetCount(uwbTxQueue);
}

int MasterComm::GetRxQueueCount()
{
    return (int)osMessageQueueGetCount(uwbRxQueue);
}

void MasterComm::ClearTxQueue()
{
    UwbTxMsg msg;
    while (osMessageQueueGet(uwbTxQueue, &msg, nullptr, 0) == osOK)
    {
        // 清空队列
    }
}

void MasterComm::ClearRxQueue()
{
    uwbRxMsg msg;
    while (osMessageQueueGet(uwbRxQueue, &msg, nullptr, 0) == osOK)
    {
        // 清空队列
    }
}

int MasterComm::Reconfigure()
{
    UwbTxMsg msg;
    msg.type = UWB_MSG_TYPE_CONFIG;
    msg.dataLen = 0;

    if (osMessageQueuePut(uwbTxQueue, &msg, 0, 100) != osOK)
    {
        return -1; // 队列满或超时
    }

    // 配置消息放入队列后，释放信号量通知通信任务
    osSemaphoreRelease(uwbTxSemaphore);

    return 0; // 成功
}

// static uint8_t rx_buffer[FRAME_LEN_MAX];
// static uint32_t status_reg = 0;
// static uint16_t frame_len = 0;
// /* Default communication configuration. */
// static dwt_config_t config = {
//     5,               // 通道号，推荐5或2，5抗干扰稍强
//     DWT_PRF_64M,     // 脉冲重复频率：64MHz 提高灵敏度（优于16M）
//     DWT_PLEN_1024,   // 前导码长度：越长越容易同步，稳定性更好（如1024）
//     DWT_PAC32,       // PAC大小：与PLEN匹配，PAC32适用于PLEN1024
//     9,               // TX前导码索引：通道5/64MHz下推荐用9
//     9,               // RX前导码索引：与TX一致
//     0,               // 使用标准SFD：标准SFD解码鲁棒性更好
//     DWT_BR_850K,     // 数据速率：110Kbps最稳定，误码率最低
//     DWT_PHRMODE_EXT, // 标准PHY头
//     (1025 + 64 - 32) // SFD超时时间：可按 PLEN + margin 设置
// };

// // UWB通信任务
// static void uwb_comm_task(void *argument)
// {
//     static const char *TAG = "uwb_comm";
//     uwb_tx_msg_t tx_msg;
//     uwb_rx_msg_t rx_msg;

//     // 初始化DW1000
//     reset_DW1000();
//     port_set_dw1000_slowrate();
//     if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
//     {
//         elog_e(TAG, "dwt_initialise failed");
//         osThreadExit();
//     }
//     port_set_dw1000_fastrate();

//     // 配置DW1000
//     dwt_configure(&config);
//     elog_i(TAG, "dwt_configure success");

//     // uint32_t device_id = dwt_readdevid();
//     // elog_i(TAG, "device_id: %08X", device_id);

//     // 启动接收模式
//     dwt_rxenable(DWT_START_RX_IMMEDIATE);

//     while (1)
//     {
//         // 等待发送信号量，确保队列中有完整的数据
//         if (osSemaphoreAcquire(uwb_txSemaphore, 0) == osOK)
//         {
//             // 从队列获取发送消息
//             if (osMessageQueueGet(uwb_txQueue, &tx_msg, NULL, 0) == osOK)
//             {
//                 switch (tx_msg.type)
//                 {
//                 case UWB_MSG_TYPE_SEND_DATA:
//                     dwt_forcetrxoff(); // 保证发送前DW1000已空闲

//                     // 发送UWB数据
//                     // DW1000会自动添加2字节CRC，所以实际写入的数据长度是用户数据长度
//                     // 但是dwt_writetxfctrl需要包含CRC的总长度
//                     dwt_writetxdata(tx_msg.data_len + 2, tx_msg.data, 0);
//                     dwt_writetxfctrl(tx_msg.data_len + 2, 0, 1);
//                     dwt_starttx(DWT_START_TX_IMMEDIATE);

//                     // 等待发送完成
//                     while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
//                     {
//                         osDelay(1);
//                     }
//                     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

//                     // 发送完成后重新启动接收
//                     dwt_rxenable(DWT_START_RX_IMMEDIATE);

//                     // elog_i(TAG, "Sent %d bytes done", tx_msg.data_len);
//                     break;

//                 case UWB_MSG_TYPE_CONFIG:
//                     // 重新配置DW1000
//                     dwt_configure(&config);
//                     dwt_rxenable(DWT_START_RX_IMMEDIATE);
//                     elog_i(TAG, "Config updated");
//                     break;

//                 case UWB_MSG_TYPE_SET_MODE:
//                     // 设置工作模式（预留接口）
//                     elog_i(TAG, "Mode set");
//                     break;

//                 default:
//                     break;
//                 }
//             }
//         }

//         // 检查是否有接收数据
//         status_reg = dwt_read32bitreg(SYS_STATUS_ID);
//         if (status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))
//         {
//             // elog_i(TAG, "status_reg: %08X", status_reg);
//             if (status_reg & SYS_STATUS_RXFCG)
//             {
//                 // 成功接收到数据
//                 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
//                 // elog_i(TAG, "frame_len: %d", frame_len);

//                 // frame_len包含2字节CRC，需要减去CRC长度得到实际数据长度
//                 if (frame_len >= 2 && frame_len <= FRAME_LEN_MAX)
//                 {
//                     dwt_readrxdata(rx_buffer, frame_len, 0);

//                     // 构造接收消息，只包含用户数据，不包含CRC
//                     rx_msg.data_len = frame_len - 2; // 减去2字节CRC
//                     for (int i = 0; i < rx_msg.data_len; i++)
//                     {
//                         rx_msg.data[i] = rx_buffer[i];
//                     }
//                     rx_msg.timestamp = osKernelGetTickCount();
//                     rx_msg.status_reg = status_reg;

//                     // 将数据放入接收队列
//                     osMessageQueuePut(uwb_rxQueue, &rx_msg, 0, 0);

//                     // 如果有回调函数，调用它
//                     if (uwb_rx_callback != NULL)
//                     {
//                         uwb_rx_callback(&rx_msg);
//                     }
//                 }

//                 // 清除接收完成标志
//                 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
//             }
//             else
//             {
//                 // 接收错误
//                 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//                 elog_e(TAG, "RX error: %08X", status_reg);
//             }

//             // 重新启动接收
//             dwt_rxenable(DWT_START_RX_IMMEDIATE);
//         }
//     }
// }