#include "nmpc_planner/keyboard_input.hpp"
#include <iostream>
#include <algorithm>

#ifdef __linux__
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <fcntl.h>
#endif

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#endif

namespace nmpc_planner {

KeyboardInput::KeyboardInput() {
#ifdef __linux__
    settings_saved_ = false;
    original_settings_ = new struct termios;
    setNonBlocking();
#endif
}

KeyboardInput::~KeyboardInput() {
#ifdef __linux__
    restoreSettings();
    if (original_settings_) {
        delete static_cast<struct termios*>(original_settings_);
    }
#endif
}

char KeyboardInput::getKey(double timeout_ms) {
#ifdef __linux__
    if (!settings_saved_) {
        return '\0';
    }
    
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    timeout.tv_sec = static_cast<long>(timeout_ms / 1000.0);
    timeout.tv_usec = static_cast<long>((timeout_ms - timeout.tv_sec * 1000.0) * 1000.0);
    
    int result = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    
    if (result > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            return key;
        }
    }
    
    return '\0';  // 超时或无输入
    
#elif defined(_WIN32)
    if (_kbhit()) {
        return _getch();
    }
    return '\0';
#else
    return '\0';  // 不支持的平台
#endif
}

std::array<double, 2> KeyboardInput::processManualControl(char key, const std::array<double, 2>& current_cmd) {
    std::array<double, 2> new_cmd = current_cmd;
    
    switch (key) {
        case 'w':  // 前进
            new_cmd[0] = clamp(current_cmd[0] + LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            break;
            
        case 'x':  // 后退
            new_cmd[0] = clamp(current_cmd[0] - LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            break;
            
        case 'a':  // 左转
            new_cmd[1] = clamp(current_cmd[1] + ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 'd':  // 右转
            new_cmd[1] = clamp(current_cmd[1] - ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 'q':  // 左前方
            new_cmd[0] = clamp(current_cmd[0] + DIAGONAL_LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            new_cmd[1] = clamp(current_cmd[1] + DIAGONAL_ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 'e':  // 右前方
            new_cmd[0] = clamp(current_cmd[0] + DIAGONAL_LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            new_cmd[1] = clamp(current_cmd[1] - DIAGONAL_ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 'z':  // 左后方
            new_cmd[0] = clamp(current_cmd[0] - DIAGONAL_LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            new_cmd[1] = clamp(current_cmd[1] + DIAGONAL_ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 'c':  // 右后方
            new_cmd[0] = clamp(current_cmd[0] - DIAGONAL_LINEAR_STEP, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            new_cmd[1] = clamp(current_cmd[1] - DIAGONAL_ANGULAR_STEP, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
            break;
            
        case 's':  // 停止
            new_cmd[0] = 0.0;
            new_cmd[1] = 0.0;
            break;
            
        default:
            // 保持当前指令不变
            break;
    }
    
    return new_cmd;
}

bool KeyboardInput::isExitKey(char key) const {
    return (key == '\x03');  // Ctrl+C
}

bool KeyboardInput::isModeSwitch(char key) const {
    return (key == 'i');  // 切换到自动模式
}

std::string KeyboardInput::getHelpText() const {
    return R"(
控制说明:
  w/x  : 前进/后退
  a/d  : 左转/右转  
  q/e  : 左前/右前
  z/c  : 左后/右后
  s    : 停止
  i    : 切换到自动模式
  Ctrl+C : 退出程序
)";
}

void KeyboardInput::setNonBlocking() {
#ifdef __linux__
    struct termios* settings = static_cast<struct termios*>(original_settings_);
    if (tcgetattr(STDIN_FILENO, settings) == 0) {
        settings_saved_ = true;
        
        struct termios new_settings = *settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;
        
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    }
#endif
}

void KeyboardInput::restoreSettings() {
#ifdef __linux__
    if (settings_saved_ && original_settings_) {
        struct termios* settings = static_cast<struct termios*>(original_settings_);
        tcsetattr(STDIN_FILENO, TCSANOW, settings);
        settings_saved_ = false;
    }
#endif
}

double KeyboardInput::clamp(double value, double min_val, double max_val) const {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace nmpc_planner
