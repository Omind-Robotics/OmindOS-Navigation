#ifndef TRAVERSABLE_EXTRACTOR_H
#define TRAVERSABLE_EXTRACTOR_H

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "trajectory_processor.h"
#include "region_growing.h"

namespace map_process {

/**
 * @brief Parameters for traversable area extraction
 */
struct ExtractionParams {
    // Trajectory sampling
    double trajectory_sample_interval;   // Distance interval for trajectory sampling (meters)
    
    // Seed point selection
    double seed_search_radius;          // Search radius around trajectory points (meters)
    double seed_height_offset;          // Height offset below robot for ground search (meters)
    
    // Region growing parameters
    RegionGrowingParams region_params;
    
    // Post-processing
    double merge_distance_threshold;    // Distance threshold for merging adjacent regions (meters)
    int min_region_size;               // Minimum size for final regions
    bool remove_outliers;              // Whether to remove statistical outliers
    
    // Ground surface constraints
    double max_ground_normal_angle;        // Maximum angle between ground normal and vertical (degrees)
    bool enable_height_filter;             // Enable/disable height filtering in ground point extraction
    double seed_height_diff_threshold;     // Maximum height difference from nearest seed point (meters)
    double seed_xy_search_radius;          // XY plane search radius for finding height-compatible seeds (meters)
    
    // Point cloud downsampling
    bool enable_voxel_downsampling;     // Enable voxel grid downsampling for large point clouds
    double voxel_size;                  // Voxel grid leaf size for downsampling (meters)
    int downsampling_threshold;         // Point count threshold to trigger downsampling
    
    // Trajectory format
    int trajectory_format;              // Format type: 1, 2, or 3

    // Debug output
    std::string debug_seed_points_file; // Optional seed-point debug PCD path
    std::string debug_ground_candidates_file; // Optional ground-candidate debug PCD path

    // Region selection
    bool require_seed_containment;       // Keep only regions containing trajectory seeds
    
    ExtractionParams() {
        trajectory_sample_interval = 1.0;
        seed_search_radius = 0.5;
        seed_height_offset = 1.0;
        merge_distance_threshold = 0.2;
        min_region_size = 100;
        remove_outliers = true;
        
        // Ground constraints defaults
        max_ground_normal_angle = 25.0;        // 25 degrees from vertical
        enable_height_filter = true;           // Enable height filtering by default
        seed_height_diff_threshold = 0.1;      // 0.1 meters from seed height
        seed_xy_search_radius = 5.0;           // 5.0 meters XY search radius
        
        // Downsampling defaults
        enable_voxel_downsampling = true;   // Enable by default
        voxel_size = 0.05;                  // 5cm voxel size
        downsampling_threshold = 1000000;   // 1M points threshold
        
        // Trajectory format default
        trajectory_format = 1;              // Format 1 by default
        debug_seed_points_file = "";
        debug_ground_candidates_file = "";
        require_seed_containment = true;
    }
};

/**
 * @brief Main class for extracting traversable areas from SLAM point cloud
 */
class TraversableExtractor {
public:
    /**
     * @brief Constructor
     */
    TraversableExtractor();
    
    /**
     * @brief Destructor
     */
    ~TraversableExtractor();
    
    /**
     * @brief Set extraction parameters
     * @param params Extraction parameters
     */
    void setParameters(const ExtractionParams& params);
    
    /**
     * @brief Load point cloud from PCD file
     * @param filename Path to PCD file
     * @return True if successful
     */
    bool loadPointCloud(const std::string& filename);
    
    /**
     * @brief Load trajectory from text file
     * @param filename Path to trajectory file
     * @return True if successful
     */
    bool loadTrajectory(const std::string& filename);
    
    /**
     * @brief Extract traversable areas
     * @return True if successful
     */
    bool extractTraversableAreas();
    
    /**
     * @brief Save traversable areas to PCD file
     * @param filename Output PCD filename
     * @return True if successful
     */
    bool saveTraversableAreas(const std::string& filename);
    
    /**
     * @brief Get the extracted traversable point cloud
     * @return Pointer to traversable areas point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTraversableAreas() const { 
        return traversable_cloud_; 
    }
    
    /**
     * @brief Get individual regions
     * @return Vector of point clouds, each representing a traversable region
     */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getRegions() const {
        return regions_;
    }
    
    /**
     * @brief Get processing statistics
     * @param total_points Total points in input cloud
     * @param traversable_points Points in traversable areas
     * @param num_regions Number of extracted regions
     */
    void getStatistics(int& total_points, int& traversable_points, int& num_regions) const;
    
private:
    
    /**
     * @brief Perform region growing segmentation using seed points to guide the process
     * @param seed_points The seed points to guide region growing
     * @return True if successful
     */
    bool performRegionGrowingWithSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr& seed_points);
    
    /**
     * @brief Filter point cloud to keep only ground-like points
     * @param cloud Input point cloud
     * @param normals Point normals
     * @param seed_points Reference seed points for height comparison
     * @return Filtered point cloud and corresponding indices
     */
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<int>> filterGroundPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& seed_points);
    
    /**
     * @brief Check if a point satisfies ground surface constraints
     * @param point The point to check
     * @param normal The point's normal vector
     * @param seed_height Height of the current seed point for comparison
     * @return True if point satisfies ground constraints
     */
    bool isGroundPoint(const pcl::PointXYZI& point, const pcl::Normal& normal, float seed_height);
    ExtractionParams params_;
    
    // Input data
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    TrajectoryProcessor trajectory_processor_;
    
    
    // Results
    pcl::PointCloud<pcl::PointXYZI>::Ptr traversable_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> regions_;
    
    /**
     * @brief Preprocess input point cloud (filtering, etc.)
     * @return True if successful
     */
    bool preprocessPointCloud();
    
    /**
     * @brief Merge nearby regions based on distance threshold
     * @param clusters Input clusters (indices)
     * @return Merged clusters
     */
    std::vector<std::vector<int>> mergeAdjacentRegions(const std::vector<std::vector<int>>& clusters);
    
    /**
     * @brief Convert cluster indices to point clouds
     * @param clusters Cluster indices
     * @return Vector of point clouds
     */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustersToPointClouds(
        const std::vector<std::vector<int>>& clusters);
    
    /**
     * @brief Combine all regions into a single point cloud
     * @return Combined point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr combineRegions();
    
    /**
     * @brief Remove statistical outliers from point cloud
     * @param cloud Input point cloud
     * @return Filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeStatisticalOutliers(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Calculate distance between two point clouds (minimum distance between any two points)
     * @param cloud1 First point cloud
     * @param cloud2 Second point cloud
     * @return Minimum distance between clouds
     */
    double calculateCloudDistance(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2);
};

} // namespace map_process

#endif // TRAVERSABLE_EXTRACTOR_H
