#ifndef NMPC_PLANNER_UTILS_HPP
#define NMPC_PLANNER_UTILS_HPP

#include <cmath>
#include <vector>
#include <array>

namespace nmpc_planner {
namespace utils {

// 数学工具函数
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// 距离计算
inline double euclideanDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// 角度差计算
inline double angleDifference(double angle1, double angle2) {
    return normalizeAngle(angle2 - angle1);
}

// 向量操作
template<typename T, size_t N>
std::array<T, N> scaleArray(const std::array<T, N>& arr, T scale) {
    std::array<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = arr[i] * scale;
    }
    return result;
}

// 打印工具
template<typename T, size_t N>
void printArray(const std::array<T, N>& arr, const std::string& name = "") {
    if (!name.empty()) {
        std::cout << name << ": ";
    }
    std::cout << "[";
    for (size_t i = 0; i < N; ++i) {
        std::cout << arr[i];
        if (i < N - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

} // namespace utils
} // namespace nmpc_planner

#endif // NMPC_PLANNER_UTILS_HPP