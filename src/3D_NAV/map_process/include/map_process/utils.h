#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace map_process {

/**
 * @brief Utility functions for map processing
 */
class Utils {
public:
    /**
     * @brief Convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    static double degToRad(double degrees);
    
    /**
     * @brief Convert radians to degrees
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    static double radToDeg(double radians);
    
    /**
     * @brief Calculate angle between two vectors
     * @param v1 First vector
     * @param v2 Second vector
     * @return Angle in radians
     */
    static double angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    
    /**
     * @brief Calculate Euclidean distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Distance
     */
    static double euclideanDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2);
    
    /**
     * @brief Apply voxel grid filter to point cloud
     * @param input Input point cloud
     * @param leaf_size Voxel size
     * @return Filtered point cloud
     */
    static pcl::PointCloud<pcl::PointXYZI>::Ptr voxelFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
        float leaf_size = 0.05f);
    
    /**
     * @brief Apply statistical outlier removal to point cloud
     * @param input Input point cloud
     * @param mean_k Number of neighbors to analyze
     * @param std_dev_mul Standard deviation multiplier
     * @return Filtered point cloud
     */
    static pcl::PointCloud<pcl::PointXYZI>::Ptr statisticalOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
        int mean_k = 50,
        double std_dev_mul = 1.0);
    
    /**
     * @brief Load point cloud from PCD file
     * @param filename Path to PCD file
     * @return Loaded point cloud, nullptr if failed
     */
    static pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(const std::string& filename);
    
    /**
     * @brief Save point cloud to PCD file
     * @param cloud Point cloud to save
     * @param filename Output filename
     * @param binary Whether to save in binary format
     * @return True if successful
     */
    static bool savePointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::string& filename,
        bool binary = true);
    
    /**
     * @brief Print point cloud information
     * @param cloud Point cloud
     * @param name Name/description of the cloud
     */
    static void printCloudInfo(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::string& name = "Point Cloud");
    
    /**
     * @brief Create a colored point cloud for visualization
     * @param cloud Input point cloud
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return Colored point cloud
     */
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColoredCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255);
    
    /**
     * @brief Split string by delimiter
     * @param str Input string
     * @param delimiter Delimiter character
     * @return Vector of split strings
     */
    static std::vector<std::string> split(const std::string& str, char delimiter);
    
    /**
     * @brief Trim whitespace from string
     * @param str Input string
     * @return Trimmed string
     */
    static std::string trim(const std::string& str);
    
    /**
     * @brief Check if file exists
     * @param filename File path
     * @return True if file exists
     */
    static bool fileExists(const std::string& filename);
    
    /**
     * @brief Get current timestamp as string
     * @return Timestamp string
     */
    static std::string getCurrentTimeString();
};

} // namespace map_process

#endif // UTILS_H
