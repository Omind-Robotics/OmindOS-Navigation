#ifndef NMPC_PLANNER_KEYBOARD_INPUT_HPP
#define NMPC_PLANNER_KEYBOARD_INPUT_HPP

#include <string>
#include <array>

namespace nmpc_planner {

class KeyboardInput {
public:
    KeyboardInput();
    ~KeyboardInput();
    
    // 获取按键（非阻塞）
    char getKey(double timeout_ms = 100.0);
    
    // 处理手动控制指令
    std::array<double, 2> processManualControl(char key, const std::array<double, 2>& current_cmd);
    
    // 检查是否为退出键
    bool isExitKey(char key) const;
    
    // 检查是否为模式切换键
    bool isModeSwitch(char key) const;
    
    // 获取帮助信息
    std::string getHelpText() const;

private:
    // Linux终端设置
    void setNonBlocking();
    void restoreSettings();
    
    // 平台相关的实现细节
#ifdef __linux__
    void* original_settings_;  // 使用void*避免包含termios.h
    bool settings_saved_;
#endif
    
    // 控制参数
    static constexpr double LINEAR_STEP = 0.08;
    static constexpr double ANGULAR_STEP = 0.25;
    static constexpr double DIAGONAL_LINEAR_STEP = 0.12;
    static constexpr double DIAGONAL_ANGULAR_STEP = 0.12;
    static constexpr double MAX_LINEAR_VEL = 0.4;
    static constexpr double MAX_ANGULAR_VEL = 0.8;
    
    // 辅助函数
    double clamp(double value, double min_val, double max_val) const;
};

} // namespace nmpc_planner

#endif // NMPC_PLANNER_KEYBOARD_INPUT_HPP
