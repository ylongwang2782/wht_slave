// #pragma once

// #include "InputAdapter.h"

// #ifndef LOCKSTATE_DEFINED
// #define LOCKSTATE_DEFINED
// enum class LockState { Unlocked, Locked };
// #endif

// class LockController {
// public:
//     // 使用依赖注入，接收InputAdapter
//     LockController(const char* name, InputAdapter& keyAdapter, InputAdapter&
//     buttonAdapter);

//     void update();
//     LockState getState() const;
//     const char* getName() const;

// private:
//     const char* name;
//     InputAdapter& keyAdapter;     // 钥匙传感器适配器
//     InputAdapter& buttonAdapter;  // 解锁按钮适配器
//     LockState state;
// };