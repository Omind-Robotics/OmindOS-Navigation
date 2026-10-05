#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
// #include <pcl/impl/pcl_base.hpp> // 
struct RsPointXYZIRT 
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,                         // 2023.10.8 uint8_t, intensity -> float, intensity
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
// template <typename PointT>
struct VelodynePointXYZIRT : pcl::PointXYZI          // : public VoxelGrid<PointT>
{
//     PCL_ADD_POINT4D

//     PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;  //  2025.03.03 被注释，解决报错：Failed to find match for field ‘timestamp‘
    // double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring)
                                   (float, time, time))

class PointCloudDownsampler
{
public:
    PointCloudDownsampler(ros::NodeHandle& nh)
    {
        // 订阅原始点云数据
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &PointCloudDownsampler::callback, this);
        // 发布降采样后的点云数据
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_point_cloud", 10);
        nh.param<float>("leafSizeX", leafSizeX, 0.1);
        nh.param<float>("leafSizeY", leafSizeY, 0.1);
        nh.param<float>("leafSizeZ", leafSizeZ, 0.1);
        nh.param<int>("stepSize", stepSize, 5);

        std::cout << "leafSizeX: "<< leafSizeX << std::endl;
        std::cout << "leafSizeY: "<< leafSizeY << std::endl;
        std::cout << "leafSizeZ: "<< leafSizeZ << std::endl;
        std::cout << "stepSize: "<< stepSize << std::endl;


    }
    template<typename T>
    bool has_nan(T point) 
    {
        // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
        // pcl remove nan not work normally
        // ROS_ERROR("Containing nan point!");
        if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) 
        {
            return true;
        } else {
            return false;
        }
    }
    template<typename T_in_p, typename T_out_p>
    void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in, const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) 
    {
        for (int point_id = 0; point_id < pc_in->points.size(); point_id+=stepSize) 
        {
            float p_x = pc_in->points[point_id].x;
            float p_y = pc_in->points[point_id].y;
            float p_z = pc_in->points[point_id].z; 
            if(p_x > -1 && p_x < 0 && p_y > -0.5 && p_y < 0.5)
            {
                pc_in->points[point_id].x = NULL;
            }

            if (has_nan(pc_in->points[point_id]) || pc_in->points[point_id].x == NULL)
            {
                continue;
            }
                
            T_out_p new_point;
            new_point.x = pc_in->points[point_id].x;
            new_point.y = pc_in->points[point_id].y;
            new_point.z = pc_in->points[point_id].z;
            new_point.intensity = pc_in->points[point_id].intensity;
            pc_out->points.push_back(new_point);
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in, const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) 
    {
        // to new pointcloud
        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); point_id+=stepSize) 
        {

            if (has_nan(pc_in->points[point_id]) || pc_in->points[point_id].x == NULL)
                continue;
            
            pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
        // to new pointcloud
        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); point_id+=stepSize) 
        {
            if (has_nan(pc_in->points[point_id]) || pc_in->points[point_id].x == NULL)
                continue;
            pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
        }
    }
    // 时序降采样
    void time_downsample(const sensor_msgs::PointCloud2ConstPtr& pcd_in)
    {

         //  使用memmcpy进行复制
         pcl::PointCloud<VelodynePointXYZIRT>::Ptr pclcloud(new pcl::PointCloud<VelodynePointXYZIRT>);
         double frame_start_time;
         std::memcpy(&frame_start_time, &pcd_in->data[18], 8);
         for(int i=0;i<pcd_in->width*pcd_in->height;i+=stepSize)//每一行中，每隔stepSize个点读一个点的数据
         {
             VelodynePointXYZIRT p;
             std::memcpy(&p.x,&pcd_in->data[26*i],4);
             std::memcpy(&p.y,&pcd_in->data[26*i+4],4);
             std::memcpy(&p.z,&pcd_in->data[26*i+8],4);
             std::memcpy(&p.intensity,&pcd_in->data[26*i+12],4);
             std::memcpy(&p.ring,&pcd_in->data[26*i+16],2);
             // std::memcpy(&p.time,&pcd_in->data[26*i+18],4);
             uint64_t timestamp_raw;
             RsPointXYZIRT p_Rs;
             std::memcpy(&p_Rs.timestamp,&pcd_in->data[26*i+18],8);
             p.time = float(p_Rs.timestamp - frame_start_time);
             pclcloud->points.push_back(p);
         }
        
         // 将点云数据绕y轴旋转90度
        Eigen::Affine3f transform = Eigen::Affine3f::Identity(); // 初始化变换矩阵为单位矩阵
        transform.translation() << 0, 0, 0;
        float angle_x = 0;
        float angle_y = M_PI/2;
        float angle_z = 0;
        transform.rotate(Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX()));
        transform.rotate(Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ()));
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_in_rs_rotated(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::transformPointCloud(*pclcloud, *pc_in_rs_rotated, transform);
        

        // 保留2m以下的点
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pclcloud_2m(new pcl::PointCloud<VelodynePointXYZIRT>);
        pcl::PassThrough<VelodynePointXYZIRT> pass;
        pass.setInputCloud(pc_in_rs_rotated);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1, 3);
        pass.filter(*pclcloud_2m);
        
        
        
        // 将PCL点云转换为ROS消息并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*pclcloud_2m, output);
        output.header = pcd_in->header;
        output.header.frame_id = "velodyne";
        std::cout << "=====================" << std::endl;
        

        std::cout << "after data size: " << output.data.size() << std::endl;
        std::cout << "after point num: " << output.width * output.height << std::endl;

        pub_.publish(output);


    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // double num = msg->width * msg->height;
        // if (num < 172800 )
        //     return;
        std::cout << "==========" << std::endl;
        std::cout << "before data size: " << msg->data.size() << std::endl;
        std::cout << "before point num: " << msg->width * msg->height << std::endl;
        // 记录当前时间
        ros::Time start_time = ros::Time::now();

        // 时序降采样
        time_downsample(msg); 

        // 记录处理时间
        std::cout << "cost time: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float leafSizeX, leafSizeY, leafSizeZ;
    int stepSize;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_downsampler");
    ros::NodeHandle nh("~");

    PointCloudDownsampler downsampler(nh);
    ros::spin();
    return 0;
}