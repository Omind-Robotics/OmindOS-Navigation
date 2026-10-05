#include <ros/ros.h>
#include <memory>
#include <iostream>

#include "nmpc_planner/local_planner.hpp"

int main(int argc, char** argv) {
    try {
        // 初始化ROS节点
        ros::init(argc, argv, "nmpc_local_planner_node", ros::init_options::NoSigintHandler);
        
        ROS_INFO("Starting NMPC Local Planner Node...");
        
        // 创建局部规划器节点
        auto planner = std::make_unique<nmpc_planner::LocalPlanner>();
        
        // 初始化并运行
        planner->initialize();
        
        // 进入ROS循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("NMPC Local Planner failed: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("NMPC Local Planner failed with unknown error");
        return 1;
    }
    
    ROS_INFO("NMPC Local Planner Node has shut down safely");
    return 0;
}