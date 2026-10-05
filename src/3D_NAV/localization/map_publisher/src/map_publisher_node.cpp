#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d/Open3D.h>
#include "open3d_conversions/open3d_conversions.h"

class MapPublisher
{
public:
    MapPublisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private)
    {
        // 读取参数
        nh_private_.param<std::string>("map_path", map_path_, "");
        nh_private_.param<double>("voxel_size", voxel_size_, 0.1);
        nh_private_.param<std::string>("map_frame_id", map_frame_id_, "map");
        nh_private_.param<double>("publish_rate", publish_rate_, 1.0);
        nh_private_.param<bool>("latch", latch_, true);

        // 创建publisher (latch=true 保证后订阅的节点也能收到)
        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/map", 1, latch_);

        ROS_INFO("Map Publisher Node Started");
        ROS_INFO("  map_path: %s", map_path_.c_str());
        ROS_INFO("  voxel_size: %.3f", voxel_size_);
        ROS_INFO("  map_frame_id: %s", map_frame_id_.c_str());
        ROS_INFO("  publish_rate: %.2f Hz", publish_rate_);
        ROS_INFO("  latch: %s", latch_ ? "true" : "false");
    }

    bool loadAndPublishMap()
    {
        // 检查文件路径
        if (map_path_.empty())
        {
            ROS_ERROR("Map path is empty! Please set ~map_path parameter.");
            return false;
        }

        // 加载点云地图
        ROS_INFO("Loading point cloud map from: %s", map_path_.c_str());
        pcd_map_ori_.reset(new open3d::geometry::PointCloud);

        if (!open3d::io::ReadPointCloud(map_path_, *pcd_map_ori_))
        {
            ROS_ERROR("Failed to read point cloud from: %s", map_path_.c_str());
            return false;
        }

        if (pcd_map_ori_->IsEmpty())
        {
            ROS_ERROR("Loaded point cloud is empty!");
            return false;
        }

        ROS_INFO("Successfully loaded point cloud with %zu points", pcd_map_ori_->points_.size());

        // 体素降采样(减少数据量)
        if (voxel_size_ > 0.0)
        {
            ROS_INFO("Downsampling point cloud with voxel size: %.3f", voxel_size_);
            pcd_map_downsampled_ = pcd_map_ori_->VoxelDownSample(voxel_size_);
            ROS_INFO("Downsampled to %zu points", pcd_map_downsampled_->points_.size());
        }
        else
        {
            pcd_map_downsampled_ = pcd_map_ori_;
            ROS_INFO("No downsampling applied (voxel_size <= 0)");
        }

        // 转换为ROS消息并发布
        publishMap();

        return true;
    }

    void publishMap()
    {
        sensor_msgs::PointCloud2 map_msg;

        // 使用open3d_conversions转换
        open3d_conversions::open3dToRos(*pcd_map_downsampled_, map_msg);

        map_msg.header.frame_id = map_frame_id_;
        map_msg.header.stamp = ros::Time::now();

        pub_map_.publish(map_msg);

        ROS_INFO("Published map with %zu points to /map topic", pcd_map_downsampled_->points_.size());
    }

    void spin()
    {
        if (publish_rate_ <= 0.0)
        {
            // 只发布一次
            ROS_INFO("Single publish mode (publish_rate <= 0)");
            ros::spin();
        }
        else
        {
            // 周期性发布
            ROS_INFO("Periodic publish mode at %.2f Hz", publish_rate_);
            ros::Rate rate(publish_rate_);

            while (ros::ok())
            {
                publishMap();
                ros::spinOnce();
                rate.sleep();
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_map_;

    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_downsampled_;

    std::string map_path_;
    double voxel_size_;
    std::string map_frame_id_;
    double publish_rate_;
    bool latch_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    MapPublisher map_publisher(nh, nh_private);

    // 加载并发布地图
    if (!map_publisher.loadAndPublishMap())
    {
        ROS_ERROR("Failed to load and publish map. Exiting...");
        return -1;
    }

    // 保持节点运行
    map_publisher.spin();

    return 0;
}
