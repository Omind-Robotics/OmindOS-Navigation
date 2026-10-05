/*********************************************************************
 *  prm_astar_planner.cpp
 *  占据体素=可通行，空体素=禁区，边界预存，起点/终点吸附，
 *  一次性建 PRM 路网，A* 查最短路径，RViz 显示网络+路径
 *  修改：通过tf监听map->motion_link变换作为实时起点
 *********************************************************************/
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include <queue>
#include <random>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <mutex>

#include "gridnn.hpp"
#include "nav_state.h"
#include "edt_3d.h"
using KdTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix3Xd,3,nanoflann::metric_L2,false>;

struct VoxelKey{ int x,y,z; };
bool operator==(const VoxelKey&a,const VoxelKey&b){
    return a.x==b.x&&a.y==b.y&&a.z==b.z;
}
namespace std{
    template<>struct hash<VoxelKey>{
        size_t operator()(const VoxelKey&k)const noexcept{
            return std::hash<int>()(k.x)^std::hash<int>()(k.y)^std::hash<int>()(k.z);
        }
    };
}

/* =========================  planner  ========================= */
class PRMAStarPlanner{
public:
    PRMAStarPlanner():nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_){
        // EDT 参数
        nh_.param("edt_xy_expand", edt_xy_expand_, 0.5);      // XY 方向扩展距离
        nh_.param("edt_z_thickness", edt_z_thickness_, 2);    // Z 方向膨胀层数

        // 机器人高度参数（用于竖向空隙检查）
        nh_.param("robot_height", robot_height_, 1.8);        // 机器人高度 (m)
        nh_.param("enable_headroom_check", enable_headroom_check_, true);  // 是否启用头顶检查

        if (enable_headroom_check_) {
            ROS_INFO("Vertical headroom check ENABLED: robot_height=%.2f m", robot_height_);
        } else {
            ROS_WARN("Vertical headroom check DISABLED - may allow paths under low obstacles");
        }

        // 安全距离参数（简化：只用一个参数控制）
        nh_.param("safe_margin", safe_margin_, 0.50);         // 安全距离阈值
        nh_.param("use_soft_penalty", use_soft_penalty_, false);
        nh_.param("voxel_leaf", voxel_leaf_, 0.2);

        // A* clearance惩罚参数
        nh_.param("clearance_penalty_weight", clearance_penalty_weight_, 5.0);  // 惩罚权重
        nh_.param("clearance_penalty_scale", clearance_penalty_scale_, 0.3);    // 指数衰减尺度
        nh_.param("use_clearance_penalty", use_clearance_penalty_, true);       // 是否启用新惩罚

        // tf相关参数
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("robot_frame", robot_frame_, "motion_link");
        nh_.param("auto_replan", auto_replan_, true);  // 是否自动重新规划
        nh_.param("replan_frequency", replan_frequency_, 1.0);  // 重新规划频率 (Hz)
        nh_.param("goal_change_threshold", goal_change_threshold_, 0.1);  // 目标点变化阈值
        nh_.param("force_replan_on_new_goal", force_replan_on_new_goal_, true);  // 新目标到达时是否强制重规划

        // 导航完成判定参数
        nh_.param("pos_tolerance", pos_tol_, 0.50);      // 位置容差 (m)
        nh_.param("yaw_tolerance", yaw_tol_, 0.14);      // 航向容差 (约8度)

        // 滞后机制参数：退出GOAL_ALIGN的位置容差阈值（应大于进入阈值）
        nh_.param("pos_tolerance_exit", pos_tol_exit_, pos_tol_ * 2.0);  // 默认为进入阈值的2倍

        /* ---- 1. 点云 ---- */
        std::string pcd_path;
        nh_.param<std::string>("pcd_path", pcd_path, "");
        if (pcd_path.empty()) {
          ROS_FATAL("Parameter ~pcd_path is required; point it at a traversable-area PCD.");
          ros::shutdown();
          return;
        }
        sad::CloudPtr cloud(new sad::PointCloudType);
        if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0 || cloud->empty()) {
          ROS_FATAL_STREAM("Unable to load traversable-area PCD: " << pcd_path);
          ros::shutdown();
          return;
        }

        /* ---- 2. 滤波 ---- */
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_leaf_,voxel_leaf_,voxel_leaf_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*cloud_f);
        pcl::toROSMsg(*cloud_f,pcd_map_);
        pcd_map_.header.frame_id="map";
        grid_.SetPointCloud(cloud_f);

        /* ---- 3. 概率体素 ---- */
        voxel_res_=voxel_leaf_; min_hits_=1;
        for(const auto&p:cloud_f->points){
            VoxelKey k{int(std::floor(p.x/voxel_res_)),
                       int(std::floor(p.y/voxel_res_)),
                       int(std::floor(p.z/voxel_res_))};
            hit_cnt_[k]++;
        }

        // 保存原始占据信息（在走廊扩展前）
        raw_hit_cnt_ = hit_cnt_;
        ROS_INFO("Saved raw occupancy map: %zu occupied voxels", raw_hit_cnt_.size());

        // 收集所有自由体素中心（用于 snapToFree 等操作）
        for(const auto&kv:hit_cnt_){
            if(kv.second<min_hits_) continue;
            Eigen::Vector3d c((kv.first.x+0.5)*voxel_res_,
                              (kv.first.y+0.5)*voxel_res_,
                              (kv.first.z+0.5)*voxel_res_);
            free_centers_.push_back(c);
        }
        ROS_INFO("Voxelization complete: %zu free voxels", free_centers_.size());

        /* ===== 3.5 使用 3D EDT 计算完整距离场 ===== */
        ROS_INFO("Computing 3D Euclidean Distance Transform...");
        ros::Time edt_start = ros::Time::now();

        // Step 1: 计算体素网格范围
        int min_x = INT_MAX, max_x = INT_MIN;
        int min_y = INT_MAX, max_y = INT_MIN;
        int min_z = INT_MAX, max_z = INT_MIN;
        for (const auto& kv : hit_cnt_) {
            min_x = std::min(min_x, kv.first.x);
            max_x = std::max(max_x, kv.first.x);
            min_y = std::min(min_y, kv.first.y);
            max_y = std::max(max_y, kv.first.y);
            min_z = std::min(min_z, kv.first.z);
            max_z = std::max(max_z, kv.first.z);
        }

        // XY 方向扩展（参数可配置）
        int xy_expand = static_cast<int>(std::ceil(edt_xy_expand_ / voxel_res_));
        min_x -= xy_expand;
        max_x += xy_expand;
        min_y -= xy_expand;
        max_y += xy_expand;

        int nx = max_x - min_x + 1;
        int ny = max_y - min_y + 1;
        int nz = max_z - min_z + 1;
        int total = nx * ny * nz;

        ROS_INFO("Grid dimensions: %d x %d x %d = %d voxels (min: [%d,%d,%d])",
                 nx, ny, nz, total, min_x, min_y, min_z);

        // Step 2: 生成二值占据数组
        std::vector<uint8_t> occupancy(total, 0);  // 默认为障碍
        auto index_fn = [&](int x, int y, int z) -> int {
            return ((z - min_z) * ny + (y - min_y)) * nx + (x - min_x);
        };

        // 标记已观测到的可通行体素
        for (const auto& kv : hit_cnt_) {
            if (kv.second >= min_hits_) {
                int idx = index_fn(kv.first.x, kv.first.y, kv.first.z);
                occupancy[idx] = 1;  // 可通行
            }
        }

        // Z 方向膨胀：为薄层结构扩展厚度（参数可配置）
        std::vector<uint8_t> occupancy_expanded = occupancy;
        for (const auto& kv : hit_cnt_) {
            if (kv.second >= min_hits_) {
                int base_x = kv.first.x;
                int base_y = kv.first.y;
                int base_z = kv.first.z;

                // 向上下扩展
                for (int dz = -edt_z_thickness_; dz <= edt_z_thickness_; ++dz) {
                    int z = base_z + dz;
                    if (z < min_z || z > max_z) continue;
                    int idx = index_fn(base_x, base_y, z);
                    occupancy_expanded[idx] = 1;
                }
            }
        }

        ROS_INFO("Occupancy grid prepared (expanded %d layers in Z)", edt_z_thickness_);

        // Step 3: 计算 3D EDT
        std::vector<float> distance_field;
        EDT3D::compute(occupancy_expanded, nx, ny, nz, voxel_res_, distance_field);

        double edt_time = (ros::Time::now() - edt_start).toSec();
        ROS_INFO("EDT computation completed in %.2f s", edt_time);

        // Step 4: 回写距离到 voxel_margin_map_
        voxel_margin_map_.clear();
        double davg = 0.0;
        int valid_cnt = 0;
        double dmin_stat = 1e9, dmax_stat = 0.0;

        for (const auto& kv : hit_cnt_) {
            if (kv.second < min_hits_) continue;

            int idx = index_fn(kv.first.x, kv.first.y, kv.first.z);
            float dist = distance_field[idx];

            // 过滤异常值
            if (dist > 1e6) dist = 0.5;  // 截断极大值

            voxel_margin_map_[kv.first] = dist;
            davg += dist;
            valid_cnt++;
            dmin_stat = std::min(dmin_stat, (double)dist);
            dmax_stat = std::max(dmax_stat, (double)dist);
        }

        davg = (valid_cnt > 0) ? davg / valid_cnt : 0.0;

        ROS_INFO("Distance field stats: min=%.3f m, max=%.3f m, avg=%.3f m",
                 dmin_stat, dmax_stat, davg);
        ROS_INFO("Safe margin threshold: %.3f m (from launch file)", safe_margin_);

        // Step 6: 根据距离阈值筛选安全体素，并进行竖向空隙过滤
        safe_centers_.clear();
        int headroom_rejected = 0;  // 统计因头顶不足被剔除的体素数量
        for (const auto& kv : voxel_margin_map_) {
            // 先检查竖向空隙
            if (!hasHeadroom(kv.first)) {
                headroom_rejected++;
                continue;
            }

            // 再检查距离阈值
            if (kv.second >= safe_margin_) {
                Eigen::Vector3d center(
                    (kv.first.x + 0.5) * voxel_res_,
                    (kv.first.y + 0.5) * voxel_res_,
                    (kv.first.z + 0.5) * voxel_res_
                );
                safe_centers_.push_back(center);
            }
        }
        ROS_INFO("Headroom filtering: %d voxels rejected (insufficient vertical clearance)",
                 headroom_rejected);

        // Step 7: 如果安全体素太少，降低阈值（但仍保留竖向检查）
        if (safe_centers_.size() < 100) {
            double fallback_margin = safe_margin_ * 0.5;
            ROS_WARN("Too few safe centers (%zu < 100), retrying with margin=%.3f m",
                     safe_centers_.size(), fallback_margin);
            safe_centers_.clear();
            int fallback_headroom_rejected = 0;
            for (const auto& kv : voxel_margin_map_) {
                // 同样先检查竖向空隙
                if (!hasHeadroom(kv.first)) {
                    fallback_headroom_rejected++;
                    continue;
                }

                // 再检查降低后的距离阈值
                if (kv.second >= fallback_margin) {
                    Eigen::Vector3d center(
                        (kv.first.x + 0.5) * voxel_res_,
                        (kv.first.y + 0.5) * voxel_res_,
                        (kv.first.z + 0.5) * voxel_res_
                    );
                    safe_centers_.push_back(center);
                }
            }
            ROS_INFO("Fallback headroom filtering: %d voxels rejected", fallback_headroom_rejected);
        }

        ROS_INFO("Final safe centers: %zu (threshold=%.3f m)",
                 safe_centers_.size(), safe_margin_);

        /* ---- 4. ROS 接口 ---- */
        pcd_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("pcd_map",1,true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("planned_path",1);
        graph_pub_= nh_.advertise<visualization_msgs::Marker>("prm_graph",1,true);
        current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose",1);

        // 导航状态发布器 (latched)
        nav_state_pub_ = nh_.advertise<std_msgs::UInt8>("/navigation_state", 1, true);
        nav_state_debug_pub_ = nh_.advertise<std_msgs::String>("/navigation_state_debug", 1, true);

        // 导航取消服务 (TODO: implement callback in later steps)
        cancel_service_ = nh_.advertiseService("/navigation/cancel",
                                              &PRMAStarPlanner::cancelNavigationCb, this);

        // 保留目标点订阅，但移除起点订阅
        goal_pose_sub_ = nh_.subscribe("goal_pose",1,&PRMAStarPlanner::goalCb,this);

        // 订阅动态障碍信息
        obs_raw_sub_ = nh_.subscribe("/obs_raw", 1, &PRMAStarPlanner::obstacleCallback, this);

        // 动态障碍参数
        nh_.param("obstacle_block_radius", obstacle_block_radius_, 0.5);  // 障碍封禁半径
        nh_.param("obstacle_timeout", obstacle_timeout_, 5.0);            // 障碍超时时间
        ROS_INFO("Dynamic obstacle parameters: block_radius=%.2f m, timeout=%.2f s",
                 obstacle_block_radius_, obstacle_timeout_);

        nh_.param("step_size",step_size_,0.5);
        nh_.param("max_nodes",max_nodes_,15000);
        nh_.param("k_neigh",k_neigh_,20);

        // 坡度约束参数
        double max_slope_deg = 30.0;  // 默认30度
        nh_.param("max_slope_deg", max_slope_deg, 30.0);
        max_slope_tan_ = std::tan(max_slope_deg * M_PI / 180.0);
        ROS_INFO("PRM slope constraint: max %.1f deg (tan=%.3f)", max_slope_deg, max_slope_tan_);

        /* ---- 5. 建 PRM ---- */
        buildPRM();
        publishGraph();          // 显示路网

        // 初始化PRM节点的KD树（用于动态障碍封禁）
        if (!vertices_.empty()) {
            // 使用成员变量存储矩阵，避免KD树内部引用悬空
            vertex_mat_.resize(3, vertices_.size());
            for (size_t i = 0; i < vertices_.size(); ++i) {
                vertex_mat_.col(i) = vertices_[i].pos;
            }
            vertex_kdtree_.reset(new KdTree(3, vertex_mat_));
            vertex_kdtree_->index_->buildIndex();

            // 初始化节点封禁状态（全部初始化为false）
            vertex_blocked_.resize(vertices_.size(), false);

            ROS_INFO("Initialized vertex KD-tree with %zu vertices for dynamic obstacle detection",
                     vertices_.size());
        }

        /* 6. 发布安全体素中心，方便 RViz 对比 */
        safe_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("safe_centers", 1, true);
        publishSafeCenters();

        // 初始化状态
        has_goal_ = false;
        last_start_pos_ = Eigen::Vector3d::Zero();
        last_goal_pos_ = Eigen::Vector3d::Zero();
        force_replan_ = false;
        goal_yaw_ = 0.0;
        last_replan_time_ = ros::Time::now();
        last_obstacle_time_ = ros::Time(0);  // 初始化为0，表示尚未收到障碍信息

        // 初始化导航状态为 WAITING 并发布
        // Note: Initialize to a different state first to ensure setNavState() publishes
        nav_state_ = NavState::ABORTED;  // Temporary initial value
        setNavState(NavState::WAITING);  // This will trigger publishing

        ROS_INFO("PRM Planner initialized. Listening for tf: %s -> %s",
                 map_frame_.c_str(), robot_frame_.c_str());
    }

    void setGoalPose(const geometry_msgs::PoseStamped& goal_pose)
    {
        goal_pose_ = goal_pose;
        has_goal_ = true;
    }

    // 新增：获取当前机器人位置的方法
    bool getCurrentPose(geometry_msgs::PoseStamped& current_pose) {
        try {
            geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
                map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.1));
            
            current_pose.header.stamp = tf_stamped.header.stamp;
            current_pose.header.frame_id = map_frame_;
            current_pose.pose.position.x = tf_stamped.transform.translation.x;
            current_pose.pose.position.y = tf_stamped.transform.translation.y;
            current_pose.pose.position.z = tf_stamped.transform.translation.z;
            current_pose.pose.orientation = tf_stamped.transform.rotation;
            
            return true;
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Could not get transform from %s to %s: %s", 
                             map_frame_.c_str(), robot_frame_.c_str(), ex.what());
            return false;
        }
    }

    // 新增：主循环更新方法（基于状态机）
    void update() {
        geometry_msgs::PoseStamped current_pose;
        if (!getCurrentPose(current_pose)) {
            return;  // 无法获取tf变换
        }

        // 发布当前位置用于调试
        current_pose_pub_.publish(current_pose);

        // 状态机分支逻辑
        switch (nav_state_) {
            case NavState::WAITING:
                // 空闲状态，等待新目标
                // goalCb会设置GLOBAL_PLANNING状态
                break;

            case NavState::GLOBAL_PLANNING:
                // 执行一次全局规划
                performGlobalPlanning(current_pose);
                break;

            case NavState::TRACKING:
                // 持续发布当前路径
                if (!last_path_.poses.empty()) {
                    path_pub_.publish(last_path_);
                }

                // 检查是否需要重新规划（基于固定频率）
                if (auto_replan_ && shouldReplan()) {
                    ROS_INFO("[Tracking] Triggering replanning (frequency-based)");
                    performGlobalPlanning(current_pose);
                } else {
                    // 检查是否接近目标（进入姿态对齐阶段）
                    checkGoalProximity(current_pose);
                }
                break;

            case NavState::GOAL_ALIGN:
                // 位置已到达，仅监控姿态误差
                checkGoalAlignment(current_pose);
                break;

            case NavState::COMPLETED:
            case NavState::ABORTED:
                // 任务结束，不做任何事
                break;
        }
    }

private:
    /* -------------------- tf相关成员 -------------------- */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;
    bool auto_replan_;
    double replan_frequency_;               // 重规划频率 (Hz)
    ros::Time last_replan_time_;            // 上次重规划时间
    
    /* -------------------- 状态管理 -------------------- */
    bool has_goal_;
    nav_msgs::Path last_path_;
    Eigen::Vector3d last_start_pos_;
    Eigen::Vector3d last_goal_pos_;         // 上次规划的目标点位置
    bool force_replan_;                     // 强制重规划标志
    double goal_change_threshold_;          // 目标点变化阈值
    bool force_replan_on_new_goal_;         // 新目标到达时是否强制重规划

    /* -------------------- 导航完成判定 -------------------- */
    double pos_tol_;                        // 位置容差 (m) - 进入GOAL_ALIGN的阈值
    double pos_tol_exit_;                   // 位置容差退出阈值 (m) - 退出GOAL_ALIGN的阈值（滞后机制）
    double yaw_tol_;                        // 航向容差 (rad)
    double goal_yaw_;                       // 缓存的目标航向 (rad)

    /* -------------------- 安全距离 -------------------- */
    double safe_margin_;                       // 离边界最小安全距离
    std::vector<Eigen::Vector3d> safe_centers_; // 安全体素中心
    std::vector<double> vertex_margin_;        // 每个 PRM 节点到边界的距离

    /* -------------------- PRM 建图 -------------------- */
    struct EdgeInfo {
        int target_id;           // 目标顶点ID
        double min_clearance;    // 边沿线最小clearance
        double avg_clearance;    // 边沿线平均clearance
    };
    struct Vertex{
        Eigen::Vector3d pos;
        std::vector<EdgeInfo> adj;  // 改为存储EdgeInfo而非int
    };
    std::vector<Vertex> vertices_;

    void buildPRM(){
        if (safe_centers_.size() < 100) {
            ROS_WARN("Too few safe centers (%zu), falling back to half-safe",
                    safe_centers_.size());
            double fall_back = 0.5;               // 50% 安全阈值
            safe_centers_.clear();
            for (const auto& kv : voxel_margin_map_)
                if (kv.second >= safe_margin_ * fall_back)
                    safe_centers_.push_back(Eigen::Vector3d(
                        (kv.first.x + 0.5) * voxel_res_,
                        (kv.first.y + 0.5) * voxel_res_,
                        (kv.first.z + 0.5) * voxel_res_));
            if (safe_centers_.empty())              // 最坏情况
            {
                ROS_ERROR("Even re-erode failed, fall back to free centers");
                safe_centers_ = free_centers_;
            }
        }
        ROS_INFO("Building PRM ...");
        ros::Time t0=ros::Time::now();

        /* 1. 随机采样 */
        std::mt19937 rng{std::random_device{}()};
        if (safe_centers_.empty()) {
            ROS_ERROR("No safe centers! Reduce safe_margin or check point cloud.");
            return;
        }
        std::uniform_int_distribution<int> dist(0,safe_centers_.size()-1);
        vertices_.reserve(max_nodes_);
        for(int i=0;i<max_nodes_;++i){
            vertices_.emplace_back();
            vertices_.back().pos=safe_centers_[dist(rng)];
        }

        /* 2. k-近邻连边（lazy 检查） */
        Eigen::Matrix3Xd mat(3,vertices_.size());
        for(size_t i=0;i<vertices_.size();++i) mat.col(i)=vertices_[i].pos;
        KdTree kdtree(3,mat);
        kdtree.index_->buildIndex();

        const double rad=step_size_*1.0;          // 距离上限
        nanoflann::SearchParameters params;
        int slope_rejected = 0;  // 统计被拒绝的边数
        int clearance_rejected = 0;  // 统计因clearance被拒绝的边数
        for(size_t i=0;i<vertices_.size();++i){
            std::vector<nanoflann::ResultItem<long int,double>> ret;
            kdtree.index_->radiusSearch(mat.col(i).data(),rad,ret,params);
            std::vector<std::pair<double,int>> tmp;
            for(auto&r:ret) if(r.first!=static_cast<long>(i))
                tmp.emplace_back(r.second,static_cast<int>(r.first));
            std::partial_sort(tmp.begin(),tmp.begin()+std::min(k_neigh_,int(tmp.size())),tmp.end());
            for(int j=0;j<std::min(k_neigh_,int(tmp.size()));++j){
                int vid=tmp[j].second;

                // 检查坡度约束
                if(!isSlopeValid(vertices_[i].pos, vertices_[vid].pos)){
                    slope_rejected++;
                    continue;
                }

                // 检查碰撞+沿线clearance
                EdgeCheckResult edge_result = isEdgeFree(vertices_[i].pos, vertices_[vid].pos);
                if(edge_result.is_free){
                    // 存储带clearance信息的边
                    EdgeInfo edge_i_to_vid;
                    edge_i_to_vid.target_id = vid;
                    edge_i_to_vid.min_clearance = edge_result.min_clearance;
                    edge_i_to_vid.avg_clearance = edge_result.avg_clearance;
                    vertices_[i].adj.push_back(edge_i_to_vid);

                    // 反向边
                    EdgeInfo edge_vid_to_i;
                    edge_vid_to_i.target_id = i;
                    edge_vid_to_i.min_clearance = edge_result.min_clearance;
                    edge_vid_to_i.avg_clearance = edge_result.avg_clearance;
                    vertices_[vid].adj.push_back(edge_vid_to_i);
                } else {
                    clearance_rejected++;
                }
            }
        }
        ROS_INFO("PRM edge filtering: %d edges rejected (slope), %d edges rejected (clearance)",
                 slope_rejected, clearance_rejected);
        ROS_INFO("PRM built: %zu vertices, elapsed %.2f s",vertices_.size(),(ros::Time::now()-t0).toSec());
    }

    /**
     * @brief 检查两点连线的坡度是否满足约束
     * @param a 起点
     * @param b 终点
     * @return true if slope is within limit, false otherwise
     */
    bool isSlopeValid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        double dz = std::abs(b.z() - a.z());  // 高度差
        double dxy = std::hypot(b.x() - a.x(), b.y() - a.y());  // 水平距离

        // 避免除零：如果水平距离极小（几乎垂直），认为坡度过大
        if (dxy < 1e-6) {
            return dz < 1e-6;  // 只有高度差也极小时才允许
        }

        double slope_tan = dz / dxy;
        return slope_tan <= max_slope_tan_;
    }

    // 边碰撞检查结果
    struct EdgeCheckResult {
        bool is_free;           // 边是否可通行
        double min_clearance;   // 沿边最小clearance
        double avg_clearance;   // 沿边平均clearance
    };

    EdgeCheckResult isEdgeFree(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
        EdgeCheckResult result;
        result.is_free = false;
        result.min_clearance = std::numeric_limits<double>::max();
        result.avg_clearance = 0.0;

        int N = stdceil((b-a).norm()/0.1);
        if (N == 0) {
            // 起点终点重合
            result.is_free = true;
            result.min_clearance = distanceToBoundary(a);
            result.avg_clearance = result.min_clearance;
            return result;
        }

        double sum_clearance = 0.0;
        for(int i=0; i<=N; ++i){
            Eigen::Vector3d p = a + (b-a)*(i/double(N));

            // 检查碰撞
            if(!isFree(p)) {
                result.is_free = false;
                return result;
            }

            // 获取当前点的clearance
            double dist = distanceToBoundary(p);

            // 检查安全距离阈值
            if (dist < safe_margin_) {
                result.is_free = false;
                return result;
            }

            // 累积clearance统计
            sum_clearance += dist;
            result.min_clearance = std::min(result.min_clearance, dist);
        }

        // 所有检查通过，边可通行
        result.is_free = true;
        result.avg_clearance = sum_clearance / (N + 1);

        return result;
    }

    static int stdceil(double x){ return int(std::ceil(x)); }

    /* -------------------- A* 查询 -------------------- */
    struct QElem{
        int id; double f;
        bool operator<(const QElem&o)const{ return f>o.f; }  // min-heap
    };
    
public:
    nav_msgs::Path computePath(const Eigen::Vector3d& start,const Eigen::Vector3d& goal){
        nav_msgs::Path empty;

        /* 1. 吸附到路网（多级尝试策略） */
        int sId = -1, gId = -1;

        // 第一次尝试：标准搜索半径 (2.0 * step_size_)，跳过被封禁节点
        sId = findClosestVertex(start, true, 2.0);
        gId = findClosestVertex(goal, true, 2.0);

        // 第二次尝试：如果失败，扩大搜索半径到 3.0 * step_size_
        if (sId < 0) {
            ROS_WARN("[Path Planning] Start vertex snap failed at 2.0*step_size, retrying with 3.0*step_size");
            sId = findClosestVertex(start, true, 3.0);
        }
        if (gId < 0) {
            ROS_WARN("[Path Planning] Goal vertex snap failed at 2.0*step_size, retrying with 3.0*step_size");
            gId = findClosestVertex(goal, true, 3.0);
        }

        // 第三次尝试：如果还是失败，忽略封禁状态（应急措施）
        if (sId < 0) {
            ROS_ERROR("[Path Planning] Start vertex snap still failed, trying without blocking check");
            sId = findClosestVertex(start, false, 3.0);
        }
        if (gId < 0) {
            ROS_ERROR("[Path Planning] Goal vertex snap still failed, trying without blocking check");
            gId = findClosestVertex(goal, false, 3.0);
        }

        if (sId < 0 || gId < 0) {
            ROS_ERROR("[Path Planning] Failed to snap to PRM graph: sId=%d, gId=%d", sId, gId);
            return empty;
        }

        ROS_INFO("[Path Planning] Snapped to vertices: start=%d, goal=%d", sId, gId);

        /* 2. 标准 A* */
        std::priority_queue<QElem> pq;
        std::vector<double> gScore(vertices_.size(),std::numeric_limits<double>::max());
        std::vector<int> cameFrom(vertices_.size(),-1);
        pq.push({sId,0});
        gScore[sId]=0;

        while(!pq.empty()){
            int u=pq.top().id; pq.pop();
            if(u==gId) break;

            // 跳过被封禁的节点
            {
                std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
                if (u >= 0 && static_cast<size_t>(u) < vertex_blocked_.size() && vertex_blocked_[u]) {
                    continue;  // 当前节点被封禁，跳过
                }
            }

            for(const EdgeInfo& edge:vertices_[u].adj){
                int v = edge.target_id;

                // 跳过被封禁的目标节点
                {
                    std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
                    if (v >= 0 && static_cast<size_t>(v) < vertex_blocked_.size() && vertex_blocked_[v]) {
                        continue;  // 目标节点被封禁，跳过这条边
                    }
                }

                double edge_len = (vertices_[v].pos - vertices_[u].pos).norm();

                // 计算边代价：根据配置选择不同的惩罚策略
                double edge_cost = edge_len;

                if (use_clearance_penalty_) {
                    // 新方案：基于边的min_clearance的激进惩罚
                    // cost = length * (1 + weight * exp(-min_clearance / scale))
                    // 当 min_clearance 很小时，惩罚指数增长
                    double penalty_factor = 1.0 + clearance_penalty_weight_ *
                                           std::exp(-edge.min_clearance / clearance_penalty_scale_);
                    edge_cost = edge_len * penalty_factor;
                } else if (use_soft_penalty_) {
                    // 旧方案：基于终点的温和惩罚（保留兼容性）
                    edge_cost = edge_len * (1.0 + 3.0 * std::exp(-distanceToBoundary(vertices_[v].pos) / 0.3));
                }

                double tentative = gScore[u] + edge_cost;
                if(tentative<gScore[v]){
                    cameFrom[v]=u;
                    gScore[v]=tentative;
                    double h=(vertices_[v].pos-goal).norm();
                    pq.push({v,tentative+h});
                }
            }
        }
        if(cameFrom[gId]<0) return empty; // 无路径

        /* 3.  reconstruct */
        std::vector<Eigen::Vector3d> pts;
        for(int at=gId;at!=-1;at=cameFrom[at]) pts.push_back(vertices_[at].pos);
        std::reverse(pts.begin(),pts.end());
        nav_msgs::Path path;
        path.header.frame_id="map";
        path.header.stamp=ros::Time::now();

        // 添加所有路径点
        for(size_t i = 0; i < pts.size(); ++i){
            geometry_msgs::PoseStamped ps;
            ps.header=path.header;
            ps.pose.position.x=pts[i].x();
            ps.pose.position.y=pts[i].y();
            ps.pose.position.z=pts[i].z() + 0.35;

            // 如果是最后几个点，写入目标姿态
            if (i >= pts.size() - 3 && has_goal_) {
                ps.pose.orientation = goal_pose_.pose.orientation;
            }
            // 否则默认姿态（后续由贝塞尔优化器计算）

            path.poses.push_back(ps);
        }

        // 追加1-2个重复的终点，强化终点姿态约束
        if (!path.poses.empty() && has_goal_) {
            geometry_msgs::PoseStamped final_pose = path.poses.back();
            final_pose.pose.orientation = goal_pose_.pose.orientation;
            path.poses.push_back(final_pose);  // 重复一次
        }

        return path;
    }

private:
    /**
     * @brief Find closest unblocked vertex to a given position
     * @param p Query position
     * @param skip_blocked If true, skip vertices blocked by dynamic obstacles
     * @param max_radius_multiplier Maximum search radius as multiplier of step_size_ (default: 2.0)
     * @return Vertex index, or -1 if not found
     */
    int findClosestVertex(const Eigen::Vector3d& p, bool skip_blocked = true,
                         double max_radius_multiplier = 2.0) const {
        int best = -1;
        double bestD = std::numeric_limits<double>::max();

        // Thread-safe access to blocking status
        std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);

        // Find closest unblocked vertex
        for(size_t i = 0; i < vertices_.size(); ++i) {
            // Skip blocked vertices if requested
            if (skip_blocked && i < vertex_blocked_.size() && vertex_blocked_[i]) {
                continue;
            }

            double d = (vertices_[i].pos - p).norm();
            if(d < bestD) {
                bestD = d;
                best = i;
            }
        }

        double max_radius = step_size_ * max_radius_multiplier;

        if (bestD < max_radius) {
            if (skip_blocked && best >= 0 && best < static_cast<int>(vertex_blocked_.size())) {
                ROS_DEBUG("[Vertex Snap] Found unblocked vertex %d at distance %.3f m (limit: %.3f m)",
                         best, bestD, max_radius);
            }
            return best;
        } else {
            if (skip_blocked) {
                ROS_WARN("[Vertex Snap] No unblocked vertex within %.3f m (closest: %.3f m at vertex %d)",
                        max_radius, bestD, best);
            } else {
                ROS_WARN("[Vertex Snap] No vertex within %.3f m (closest: %.3f m)",
                        max_radius, bestD);
            }
            return -1;
        }
    }

    /* -------------------- ROS 回调 -------------------- */
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        // 如果当前正在执行任务，先发送ABORTED再进入WAITING
        if (nav_state_ != NavState::WAITING && nav_state_ != NavState::COMPLETED && nav_state_ != NavState::ABORTED) {
            ROS_INFO("[Goal] Aborting current task before accepting new goal");
            setNavState(NavState::ABORTED);
            ros::Duration(0.05).sleep();  // 短暂延迟确保状态发布
            setNavState(NavState::WAITING);
            ros::Duration(0.05).sleep();
        }

        // 保存新目标
        goal_pose_ = *msg;
        snapPose(goal_pose_);
        has_goal_ = true;

        // 提取并缓存目标航向
        goal_yaw_ = quaternionToYaw(goal_pose_.pose.orientation);

        // 清空历史路径和状态
        last_path_.poses.clear();
        force_replan_ = false;

        ROS_INFO("[Goal] New goal received: pos=(%.2f, %.2f, %.2f), yaw=%.2f deg",
                 goal_pose_.pose.position.x,
                 goal_pose_.pose.position.y,
                 goal_pose_.pose.position.z,
                 goal_yaw_ * 180.0 / M_PI);

        // 启动全局规划
        setNavState(NavState::GLOBAL_PLANNING);
    }

    /**
     * @brief 动态障碍回调函数
     * 接收 /obs_raw 话题的障碍信息，更新动态障碍体素集合，并封禁受影响的PRM节点
     */
    void obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);

        // 清空之前的动态障碍（每次重建）
        dynamic_blocked_voxels_.clear();

        // 解析Float32MultiArray：每3个数为一个障碍点的xyz坐标
        const std::vector<float>& data = msg->data;
        if (data.size() % 3 != 0) {
            ROS_WARN_THROTTLE(1.0, "[Obstacle] Invalid /obs_raw data size: %zu (not multiple of 3)",
                             data.size());
            return;
        }

        size_t num_obstacles = data.size() / 3;
        for (size_t i = 0; i < num_obstacles; ++i) {
            float x = data[i * 3];
            float y = data[i * 3 + 1];
            float z = data[i * 3 + 2];

            // 转换为VoxelKey
            VoxelKey k{int(std::floor(x / voxel_res_)),
                       int(std::floor(y / voxel_res_)),
                       int(std::floor(z / voxel_res_))};
            dynamic_blocked_voxels_.insert(k);
        }

        // 更新时间戳
        last_obstacle_time_ = ros::Time::now();

        ROS_INFO_THROTTLE(2.0, "[Obstacle] Received %zu obstacle points, %zu unique voxels blocked",
                         num_obstacles, dynamic_blocked_voxels_.size());

        // 使用KD树查找受影响的PRM节点
        if (!vertex_kdtree_ || vertices_.empty()) {
            ROS_WARN_THROTTLE(5.0, "[Obstacle] Vertex KD-tree not initialized, skipping node blocking");
            return;
        }

        // 重置所有节点的封禁状态
        std::fill(vertex_blocked_.begin(), vertex_blocked_.end(), false);

        // 对每个障碍点，查找附近的PRM节点并封禁
        int blocked_count = 0;
        nanoflann::SearchParameters params;
        const double search_radius_sq = obstacle_block_radius_ * obstacle_block_radius_;

        for (size_t i = 0; i < num_obstacles; ++i) {
            // 提取障碍点坐标并验证有效性
            float x = data[i * 3];
            float y = data[i * 3 + 1];
            float z = data[i * 3 + 2];

            // 跳过无效点
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            // 构造查询点（使用double数组，因为KD树期望double*）
            double query_pt[3] = {static_cast<double>(x),
                                  static_cast<double>(y),
                                  static_cast<double>(z)};

            // 在obstacle_block_radius_半径内查找节点
            std::vector<nanoflann::ResultItem<long int, double>> ret;
            vertex_kdtree_->index_->radiusSearch(query_pt, search_radius_sq, ret, params);

            // 封禁命中的节点
            for (const auto& item : ret) {
                size_t vid = static_cast<size_t>(item.first);
                if (vid < vertex_blocked_.size() && !vertex_blocked_[vid]) {
                    vertex_blocked_[vid] = true;
                    blocked_count++;
                }
            }
        }

        ROS_INFO_THROTTLE(2.0, "[Obstacle] Blocked %d PRM vertices (radius=%.2f m)",
                         blocked_count, obstacle_block_radius_);

        // 如果当前正在TRACKING状态，触发重规划
        if (nav_state_ == NavState::TRACKING && blocked_count > 0) {
            ROS_INFO("[Obstacle] Detected %d blocked vertices, triggering replanning", blocked_count);
            force_replan_ = true;
        }
    }

    /**
     * @brief Cancel navigation service callback
     * Aborts current navigation task and returns to WAITING state
     */
    bool cancelNavigationCb(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& res) {
        ROS_INFO("[Navigation] Cancel service called");

        // 如果当前有任务在执行，取消它
        if (nav_state_ != NavState::WAITING && nav_state_ != NavState::COMPLETED && nav_state_ != NavState::ABORTED) {
            // 先进入ABORTED状态
            setNavState(NavState::ABORTED);

            // 清空任务状态
            has_goal_ = false;
            last_path_.poses.clear();

            // 短暂延迟后进入WAITING
            ros::Duration(0.05).sleep();
            setNavState(NavState::WAITING);

            res.success = true;
            res.message = "Navigation cancelled successfully";
            ROS_INFO("[Navigation] Task cancelled, returned to WAITING");
        } else {
            res.success = true;
            res.message = "No active navigation task to cancel";
            ROS_INFO("[Navigation] No active task, already in idle state");
        }

        return true;
    }
    
    void snapPose(geometry_msgs::PoseStamped& p){
        Eigen::Vector3d snapped=snapToFree(poseToEigen(p));
        p.pose.position.x=snapped.x();
        p.pose.position.y=snapped.y();
        p.pose.position.z=snapped.z();
    }
    Eigen::Vector3d poseToEigen(const geometry_msgs::PoseStamped& p) const {
        return Eigen::Vector3d(p.pose.position.x,p.pose.position.y,p.pose.position.z);
    }

    /**
     * @brief Extract yaw angle from quaternion
     * @param q Quaternion
     * @return Yaw angle in radians [-π, π]
     */
    double quaternionToYaw(const geometry_msgs::Quaternion& q) const {
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    /**
     * @brief Normalize angle to [-π, π]
     */
    double normalizeAngle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief Check if replanning is needed based on fixed frequency
     * @return true if enough time has passed since last replanning
     */
    bool shouldReplan() {
        if (!has_goal_) return false;

        ros::Time now = ros::Time::now();
        double time_since_last_replan = (now - last_replan_time_).toSec();
        double replan_interval = 1.0 / replan_frequency_;  // 计算周期 (秒)

        // 定期输出调试信息
        ROS_INFO_THROTTLE(3.0, "[Replan Check] Time since last replan: %.3f s (interval: %.3f s, freq: %.2f Hz)",
                         time_since_last_replan, replan_interval, replan_frequency_);

        if (time_since_last_replan >= replan_interval) {
            ROS_INFO("[Replan Check] Time interval reached (%.3f s >= %.3f s) - triggering replan",
                      time_since_last_replan, replan_interval);
            last_replan_time_ = now;  // 更新时间戳
            return true;
        }
        return false;
    }

    /**
     * @brief Execute global planning once
     */
    void performGlobalPlanning(const geometry_msgs::PoseStamped& current_pose) {
        if (!has_goal_) {
            ROS_WARN("[Global Planning] No goal set, aborting");
            setNavState(NavState::ABORTED);
            return;
        }

        // 吸附当前位置到安全区域
        geometry_msgs::PoseStamped snapped_pose = current_pose;
        snapPose(snapped_pose);

        Eigen::Vector3d start = poseToEigen(snapped_pose);
        Eigen::Vector3d goal = poseToEigen(goal_pose_);

        // 执行PRM/A*规划
        nav_msgs::Path path = computePath(start, goal);

        if (!path.poses.empty()) {
            // 规划成功
            last_path_ = path;
            last_start_pos_ = poseToEigen(current_pose);
            last_goal_pos_ = goal;
            last_replan_time_ = ros::Time::now();  // 更新重规划时间戳
            path_pub_.publish(path);

            ROS_INFO("[Global Planning] Path found with %zu waypoints", path.poses.size());
            setNavState(NavState::TRACKING);
        } else {
            // 规划失败
            ROS_ERROR("[Global Planning] Failed to find path!");
            setNavState(NavState::ABORTED);
            // 清空状态
            has_goal_ = false;
            last_path_.poses.clear();
        }
    }

    /**
     * @brief Check if robot is close enough to goal position to start orientation alignment
     */
    void checkGoalProximity(const geometry_msgs::PoseStamped& current_pose) {
        if (!has_goal_) return;

        Eigen::Vector3d current_pos = poseToEigen(current_pose);
        Eigen::Vector3d goal_pos = poseToEigen(goal_pose_);

        double pos_err = (goal_pos - current_pos).norm();

        // 定期输出距离信息用于调试
        ROS_INFO_THROTTLE(2.0, "[Goal Proximity] Distance to goal: %.3f m (threshold: %.3f m)",
                         pos_err, pos_tol_);

        if (pos_err < pos_tol_) {
            ROS_INFO("[Goal Proximity] Position reached (error: %.3f m), entering GOAL_ALIGN", pos_err);
            setNavState(NavState::GOAL_ALIGN);
        }
    }

    /**
     * @brief Check if robot orientation matches goal orientation
     *
     * 使用滞后机制 + 状态锁定策略：
     * - 进入GOAL_ALIGN: pos_err < pos_tol_ (例如 0.5m)
     * - 退出GOAL_ALIGN: pos_err > pos_tol_exit_ (例如 1.0m，严重偏离才退出)
     * - 这避免了在边界附近由于定位噪声导致的状态抖动
     */
    void checkGoalAlignment(const geometry_msgs::PoseStamped& current_pose) {
        if (!has_goal_) return;

        // 检查位置误差
        Eigen::Vector3d current_pos = poseToEigen(current_pose);
        Eigen::Vector3d goal_pos = poseToEigen(goal_pose_);
        double pos_err = (goal_pos - current_pos).norm();

        // 检查航向误差
        double current_yaw = quaternionToYaw(current_pose.pose.orientation);
        double yaw_err = std::abs(normalizeAngle(goal_yaw_ - current_yaw));

        // 定期输出调试信息
        ROS_INFO_THROTTLE(1.0, "[Goal Alignment] pos_err=%.3f m (exit_threshold=%.3f m), yaw_err=%.3f deg",
                         pos_err, pos_tol_exit_, yaw_err * 180.0 / M_PI);

        // 判断1：姿态已对齐 → COMPLETED
        if (yaw_err < yaw_tol_) {
            ROS_INFO("[Goal Alignment] Goal reached! pos_err=%.3f m, yaw_err=%.3f deg",
                     pos_err, yaw_err * 180.0 / M_PI);
            setNavState(NavState::COMPLETED);

            // 清空任务状态
            has_goal_ = false;
            last_path_.poses.clear();
            return;
        }

        // 判断2：位置严重偏离（使用退出阈值，实现滞后机制） → ABORTED
        // 只有当偏离距离超过 pos_tol_exit_ 时才认为出现异常
        if (pos_err > pos_tol_exit_) {
            ROS_WARN("[Goal Alignment] Position severely deviated (%.3f m > %.3f m threshold), ABORTED",
                     pos_err, pos_tol_exit_);
            setNavState(NavState::ABORTED);

            // 清空任务状态
            has_goal_ = false;
            last_path_.poses.clear();
            return;
        }

        // 判断3：其他情况（pos_tol_ < pos_err <= pos_tol_exit_）
        // 保持在 GOAL_ALIGN 状态，继续旋转对齐
        // 这就是"状态锁定"：一旦进入对齐状态，只要不是严重偏离，就坚持完成对齐任务
    }

    Eigen::Vector3d snapToFree(const Eigen::Vector3d& in) const {
        if(free_centers_.empty()) return in;
        return *std::min_element(free_centers_.begin(),free_centers_.end(),
                               [&](const Eigen::Vector3d& a,const Eigen::Vector3d& b){
                                   return (a-in).norm()<(b-in).norm();});
    }
    bool isFree(const Eigen::Vector3d& pos) const {
        VoxelKey k{int(std::floor(pos.x()/voxel_res_)),
                   int(std::floor(pos.y()/voxel_res_)),
                   int(std::floor(pos.z()/voxel_res_))};

        // 首先检查静态地图：体素必须存在且命中次数足够
        auto it=hit_cnt_.find(k);
        if (it == hit_cnt_.end() || it->second < min_hits_) {
            return false;  // 不在自由空间中
        }

        // 然后检查动态障碍：如果体素被标记为动态障碍，则不自由
        // 使用线程安全的检查
        std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
        if (dynamic_blocked_voxels_.find(k) != dynamic_blocked_voxels_.end()) {
            return false;  // 体素被动态障碍占据
        }

        return true;  // 静态自由且无动态障碍
    }

    double distanceToBoundary(const Eigen::Vector3d& p) const {
        VoxelKey k{int(std::floor(p.x()/voxel_res_)),
                int(std::floor(p.y()/voxel_res_)),
                int(std::floor(p.z()/voxel_res_))};
        auto it = voxel_margin_map_.find(k);
        return (it != voxel_margin_map_.end()) ? it->second : 0.0;
    }

    /**
     * @brief 检查体素上方是否有足够的竖向空隙（机器人头顶高度）
     * @param voxel 待检查的体素坐标
     * @return true if 头顶空间充足, false if 头顶被占据
     */
    bool hasHeadroom(const VoxelKey& voxel) const {
        if (!enable_headroom_check_) {
            return true;  // 禁用检查时默认通过
        }

        // 计算需要检查的层数
        int layers_to_check = static_cast<int>(std::ceil(robot_height_ / voxel_res_));

        // 从当前体素的上一层开始检查（当前体素自身已经是自由的）
        for (int dz = 1; dz <= layers_to_check; ++dz) {
            VoxelKey upper_voxel{voxel.x, voxel.y, voxel.z + dz};

            // 检查原始占据图中是否存在障碍物
            auto it = raw_hit_cnt_.find(upper_voxel);
            if (it != raw_hit_cnt_.end() && it->second >= min_hits_) {
                // 发现头顶有障碍物
                return false;
            }
        }

        // 头顶空间充足
        return true;
    }
    // erodeVoxels() 函数已废弃：现在直接使用 3D EDT 计算的距离场进行筛选
    // 不再需要基于 XY 圆盘的形态学腐蚀操作

    /* -------------------- RViz 显示 PRM 网络 -------------------- */
    void publishGraph(){
        visualization_msgs::Marker mk;
        mk.header.frame_id="map";
        mk.header.stamp=ros::Time::now();
        mk.ns="prm"; mk.id=0; mk.type=mk.LINE_LIST;
        mk.action=mk.ADD;
        mk.pose.orientation.w=1;
        mk.scale.x=0.05;   // 线宽
        mk.color.r=1.0; mk.color.g=0.7; mk.color.b=0.0; mk.color.a=0.6;
        for(size_t i=0;i<vertices_.size();++i){
            for(const EdgeInfo& edge:vertices_[i].adj){
                int j = edge.target_id;
                if(j<static_cast<int>(i)) continue; // 无向图去重
                geometry_msgs::Point p1,p2;
                p1.x=vertices_[i].pos.x(); p1.y=vertices_[i].pos.y(); p1.z=vertices_[i].pos.z();
                p2.x=vertices_[j].pos.x(); p2.y=vertices_[j].pos.y(); p2.z=vertices_[j].pos.z();
                mk.points.push_back(p1); mk.points.push_back(p2);
            }
        }
        graph_pub_.publish(mk);
    }

    void publishSafeCenters(){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (const auto& p : safe_centers_){
            pcl::PointXYZRGB pt;
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            pt.r = 0; pt.g = 255; pt.b = 0;   // 亮绿色
            cloud.push_back(pt);
        }
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(cloud, out);
        out.header.frame_id = "map";
        out.header.stamp = ros::Time::now();
        safe_pub_.publish(out);
    }

    /* -------------------- 导航状态管理方法 -------------------- */
    /**
     * @brief Set navigation state and publish to topics
     * @param new_state The new navigation state
     */
    void setNavState(NavState new_state) {
        if (nav_state_ == new_state) {
            return;  // No change, skip publishing
        }

        nav_state_ = new_state;

        // Publish state as UInt8
        std_msgs::UInt8 state_msg;
        state_msg.data = static_cast<uint8_t>(nav_state_);
        nav_state_pub_.publish(state_msg);

        // Publish debug string
        std_msgs::String debug_msg;
        debug_msg.data = navStateToString(nav_state_);
        nav_state_debug_pub_.publish(debug_msg);

        // Log state change
        ROS_INFO("[NavState] Transitioned to: %s", navStateToString(nav_state_).c_str());
    }

    /* -------------------- 成员 -------------------- */
    ros::NodeHandle nh_;
    ros::Subscriber goal_pose_sub_;  // 只保留目标点订阅

    /* -------------------- 导航状态管理 -------------------- */
    NavState nav_state_;                    // 当前导航状态
    ros::Publisher nav_state_pub_;          // 状态发布器 (UInt8, latched)
    ros::Publisher nav_state_debug_pub_;    // 状态调试发布器 (String)
    ros::ServiceServer cancel_service_;     // 取消服务 (TODO: implement in later step)

public:
    ros::Publisher  path_pub_,pcd_pub_,graph_pub_;
    ros::Publisher safe_pub_;   // 安全点云
    ros::Publisher current_pose_pub_;  // 当前位置发布
    sensor_msgs::PointCloud2 pcd_map_;
    
private:
    geometry_msgs::PoseStamped goal_pose_;
    sad::GridNN<3> grid_;
    double voxel_res_,step_size_;
    int max_nodes_,min_hits_,k_neigh_;
    std::unordered_map<VoxelKey,int> hit_cnt_;
    std::unordered_map<VoxelKey,int> raw_hit_cnt_;  // 原始占据信息（走廊扩展前）
    std::vector<Eigen::Vector3d> free_centers_;
    // free_boundary_ 已废弃：不再使用 XY 平面 8 邻域边界标记
    std::unordered_map<VoxelKey,double> voxel_margin_map_; // <体素,3D EDT距离>

    // EDT 参数
    double edt_xy_expand_;     // XY 方向边界扩展距离 (m)
    int edt_z_thickness_;      // Z 方向膨胀层数（为薄层结构提供厚度）

    // 竖向空隙检查参数
    double robot_height_;          // 机器人高度 (m)
    bool enable_headroom_check_;   // 是否启用头顶空隙检查

    // 安全距离参数
    bool use_soft_penalty_;
    double voxel_leaf_;
    double max_slope_tan_;  // 最大坡度的tan值

    // A* clearance惩罚参数
    double clearance_penalty_weight_;   // 惩罚权重 w
    double clearance_penalty_scale_;    // 指数衰减尺度 α
    bool use_clearance_penalty_;        // 是否启用新的clearance惩罚

    /* -------------------- 动态障碍处理 -------------------- */
    ros::Subscriber obs_raw_sub_;                          // /obs_raw 订阅者
    std::unordered_set<VoxelKey> dynamic_blocked_voxels_;  // 动态障碍体素集合
    std::vector<bool> vertex_blocked_;                     // PRM节点封禁状态
    mutable std::mutex dynamic_obstacle_mutex_;            // 线程安全互斥锁（mutable以在const函数中使用）
    Eigen::Matrix3Xd vertex_mat_;                          // PRM节点坐标矩阵（KD树需要持有引用）
    std::unique_ptr<KdTree> vertex_kdtree_;                // PRM节点KD树（用于障碍检测时快速查找）
    double obstacle_block_radius_;                         // 障碍物封禁半径
    ros::Time last_obstacle_time_;                         // 上次收到障碍信息的时间
    double obstacle_timeout_;                              // 障碍信息超时时间（秒）
};

/* -------------------- main -------------------- */
int main(int argc,char** argv){
    ros::init(argc,argv,"prm_astar_planner");
    PRMAStarPlanner planner;
    
    ros::Rate rate(10);  // 10Hz 更新频率
    while(ros::ok()){
        // 发布地图点云
        planner.pcd_pub_.publish(planner.pcd_map_);
        
        // 更新规划器状态（检查tf变换并重新规划）
        planner.update();
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
