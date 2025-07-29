// #include "LockController.h"

// LockController::LockController(const char* name, InputAdapter& keyAdapter,
// InputAdapter& buttonAdapter)
//     : name(name), keyAdapter(keyAdapter), buttonAdapter(buttonAdapter),
//     state(LockState::Unlocked) {
// }

// void LockController::update() {
//     // 更新按钮边沿状态
//     keyAdapter.updateButton();
//     buttonAdapter.updateButton();

//     // 钥匙插入 → 上锁（打开阀门）
//     if (keyAdapter.wasButtonPressed()) {
//         keyAdapter.openValve();
//         state = LockState::Locked;
//     }

//     // 解锁按钮按下 → 解锁（关闭阀门）
//     if (buttonAdapter.wasButtonPressed()) {
//         keyAdapter.closeValve();
//         state = LockState::Unlocked;
//     }
// }

// LockState LockController::getState() const {
//     return state;
// }

// const char* LockController::getName() const {
//     return name;
// }