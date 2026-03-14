#include "mode_manager.hpp"
#include <iostream>

ModeManager::ModeManager()
{
    current_mode_ = Mode::IDLE;
}

void ModeManager::updateFromKeyboard(int key)
{
    switch (key)
    {
        case '0':
            current_mode_ = Mode::IDLE;
            std::cout << "[Mode] Switch to IDLE\n";
            break;

        case '1':
            current_mode_ = Mode::AUTO_AIM;
            std::cout << "[Mode] Switch to AUTO_AIM\n";
            break;

        case '2':
            current_mode_ = Mode::CALIB;
            std::cout << "[Mode] Switch to CALIB\n";
            break;

        default:
            break;
    }
}

Mode ModeManager::getMode() const
{
    return current_mode_;
}