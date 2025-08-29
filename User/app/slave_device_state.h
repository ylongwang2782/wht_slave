#pragma once

namespace SlaveApp {

/**
 * 从机设备状态枚举
 */
enum class SlaveDeviceState {
    IDLE,                   // 空闲状态
    READY,                  // 已配置状态
    RUNNING,                // 运行状态
    COLLECTING,             // 采集中状态
    COLLECTION_COMPLETE,    // 采集完成状态
    DEV_ERR                 // 错误状态
};

}    // namespace SlaveApp