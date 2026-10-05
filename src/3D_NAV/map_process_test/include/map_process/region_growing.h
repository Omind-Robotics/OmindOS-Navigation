#ifndef REGION_GROWING_H
#define REGION_GROWING_H

#include <vector>
#include <queue>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>

namespace map_process {

/**
 * @brief Parameters for region growing algorithm
 */
struct RegionGrowingParams {
    // Geometric constraints
    double normal_angle_threshold;      // Maximum angle between normals (degrees)
    double curvature_threshold;         // Maximum curvature difference
    double height_threshold;            // Maximum height difference (meters)
    double search_radius;               // Search radius for neighbors (meters)
    
    // Region constraints
    int min_cluster_size;               // Minimum points in a region
    int max_cluster_size;               // Maximum points in a region
    
    // Normal estimation parameters
    double normal_search_radius;        // Radius for normal estimation
    
    RegionGrowingParams() {
        normal_angle_threshold = 30.0;   // degrees
        curvature_threshold = 1.0;
        height_threshold = 0.1;          // meters
        search_radius = 0.3;             // meters
        min_cluster_size = 50;
        max_cluster_size = 1000000;
        normal_search_radius = 0.2;      // meters
    }
};

/**
 * @brief Custom region growing class for traversable area extraction
 */
class TraversableRegionGrowing {
public:
    /**
     * @brief Constructor
     */
    TraversableRegionGrowing();
    
    /**
     * @brief Destructor
     */
    ~TraversableRegionGrowing();
    
    /**
     * @brief Set parameters for region growing
     * @param params Region growing parameters
     */
    void setParameters(const RegionGrowingParams& params);
    
    /**
     * @brief Set input point cloud
     * @param cloud Input point cloud
     */
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Set seed points for region growing
     * @param seeds Seed points cloud
     */
    void setSeedPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& seeds);
    
    /**
     * @brief Perform region growing segmentation
     * @param clusters Output clusters (each cluster is a vector of point indices)
     * @return True if successful
     */
    bool extract(std::vector<std::vector<int>>& clusters);
    
    /**
     * @brief Get the normals computed for the input cloud
     * @return Pointer to normals
     */
    pcl::PointCloud<pcl::Normal>::Ptr getNormals() const { return normals_; }
    
private:
    RegionGrowingParams params_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_;
    
    /**
     * @brief Estimate normals for the input cloud
     * @return True if successful
     */
    bool estimateNormals();
    
    /**
     * @brief Check if two points can be merged based on geometric criteria
     * @param idx1 Index of first point
     * @param idx2 Index of second point
     * @return True if points can be merged
     */
    bool canMerge(int idx1, int idx2) const;
    
    /**
     * @brief Find the closest point in input cloud to a seed point
     * @param seed_point Seed point
     * @return Index of closest point in input cloud, -1 if not found
     */
    int findClosestPoint(const pcl::PointXYZI& seed_point) const;
    
    /**
     * @brief Grow region from a seed point
     * @param seed_idx Index of seed point
     * @param visited Set of already visited points
     * @param cluster Output cluster indices
     * @return Number of points added to cluster
     */
    int growRegion(int seed_idx, std::unordered_set<int>& visited, std::vector<int>& cluster);
    
    /**
     * @brief Check if a point satisfies traversability constraints
     * @param idx Point index
     * @return True if point is potentially traversable
     */
    bool isTraversable(int idx) const;
};

} // namespace map_process

#endif // REGION_GROWING_H
