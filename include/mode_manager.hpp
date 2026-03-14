#ifndef MODE_MANAGER_HPP
#define MODE_MANAGER_HPP

#include <atomic>

enum class Mode
{
    IDLE = 0,
    AUTO_AIM,
    CALIB
};

class ModeManager
{
public:
    ModeManager();

    void updateFromKeyboard(int key);   // 键盘更新模式
    Mode getMode() const;               // 获取当前模式

private:
    std::atomic<Mode> current_mode_;    // 原子防止多线程冲突
};

#endif