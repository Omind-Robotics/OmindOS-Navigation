#include <ros/ros.h>
#include <signal.h>
#include <memory>
#include <iostream>

#include "nmpc_planner/controller_node.hpp"

int main(int argc, char** argv) {
    try {
        // 初始化ROS节点
        ros::init(argc, argv, "nmpc_controller_node", ros::init_options::NoSigintHandler);
        
        ROS_INFO("Starting NMPC Controller Node...");
        
        // 创建控制器节点
        auto controller = std::make_unique<nmpc_planner::ControllerNode>();
        
        // 运行控制器
        controller->run();
        
    } catch (const std::exception& e) {
        ROS_ERROR("NMPC Controller failed: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("NMPC Controller failed with unknown error");
        return 1;
    }
    
    ROS_INFO("NMPC Controller Node has shut down safely");
    return 0;
}