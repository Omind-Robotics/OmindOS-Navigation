#include "map_process/traversable_extractor.h"
#include "map_process/utils.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

namespace map_process {

TraversableExtractor::TraversableExtractor() 
    : input_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
    , traversable_cloud_(new pcl::PointCloud<pcl::PointXYZI>) {
}

TraversableExtractor::~TraversableExtractor() {
}

void TraversableExtractor::setParameters(const ExtractionParams& params) {
    params_ = params;
    
    // Set trajectory format in the trajectory processor
    trajectory_processor_.setTrajectoryFormat(params.trajectory_format);
}

bool TraversableExtractor::loadPointCloud(const std::string& filename) {
    input_cloud_ = Utils::loadPointCloud(filename);
    
    if (!input_cloud_ || input_cloud_->empty()) {
        std::cerr << "Failed to load point cloud from: " << filename << std::endl;
        return false;
    }
    
    Utils::printCloudInfo(input_cloud_, "Loaded Point Cloud");
    
    return true;
}

bool TraversableExtractor::loadTrajectory(const std::string& filename) {
    return trajectory_processor_.loadTrajectory(filename);
}



bool TraversableExtractor::extractTraversableAreas() {
    std::cout << "\n=== Starting Traversable Area Extraction ===" << std::endl;
    
    // Step 1: Preprocess point cloud
    std::cout << "\nStep 1: Preprocessing point cloud..." << std::endl;
    if (!preprocessPointCloud()) {
        std::cerr << "Point cloud preprocessing failed" << std::endl;
        return false;
    }
    
    // Step 2: Sample trajectory and find seed points
    std::cout << "\nStep 2: Sampling trajectory and finding seed points..." << std::endl;
    std::vector<RobotPose> sampled_poses = trajectory_processor_.sampleTrajectory(
        params_.trajectory_sample_interval);
    
    if (sampled_poses.empty()) {
        std::cerr << "No poses sampled from trajectory" << std::endl;
        return false;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points = trajectory_processor_.findSeedPoints(
        input_cloud_, sampled_poses, 
        params_.seed_search_radius, params_.seed_height_offset);
    
    if (!seed_points || seed_points->empty()) {
        std::cerr << "No seed points found" << std::endl;
        return false;
    }
    
    if (!params_.debug_seed_points_file.empty()) {
        try {
            if (Utils::savePointCloud(seed_points, params_.debug_seed_points_file)) {
                std::cout << "Saved " << seed_points->size()
                          << " seed points to: " << params_.debug_seed_points_file << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to save seed points to "
                      << params_.debug_seed_points_file << ": " << e.what() << std::endl;
        }
    }
    
    // Step 3: Perform region growing segmentation
    std::cout << "\nStep 3: Performing region growing segmentation..." << std::endl;
    
    if (!performRegionGrowingWithSeeds(seed_points)) {
        std::cerr << "Failed to perform region growing segmentation" << std::endl;
        return false;
    }
    
    return true;
}

bool TraversableExtractor::saveTraversableAreas(const std::string& filename) {
    if (!traversable_cloud_ || traversable_cloud_->empty()) {
        std::cerr << "No traversable areas to save" << std::endl;
        return false;
    }
    
    return Utils::savePointCloud(traversable_cloud_, filename);
}

void TraversableExtractor::getStatistics(int& total_points, int& traversable_points, int& num_regions) const {
    total_points = input_cloud_ ? static_cast<int>(input_cloud_->size()) : 0;
    traversable_points = traversable_cloud_ ? static_cast<int>(traversable_cloud_->size()) : 0;
    num_regions = static_cast<int>(regions_.size());
}

bool TraversableExtractor::preprocessPointCloud() {
    if (!input_cloud_ || input_cloud_->empty()) {
        std::cerr << "No input cloud to preprocess" << std::endl;
        return false;
    }
    
    std::cout << "Original point cloud: " << input_cloud_->size() << " points" << std::endl;
    
    // Remove NaN and infinite values first
    pcl::PointCloud<pcl::PointXYZI>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    clean_cloud->reserve(input_cloud_->size());
    
    int nan_count = 0;
    for (const auto& point : input_cloud_->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            clean_cloud->points.push_back(point);
        } else {
            nan_count++;
        }
    }
    
    clean_cloud->width = clean_cloud->points.size();
    clean_cloud->height = 1;
    clean_cloud->is_dense = true;
    
    std::cout << "Removed " << nan_count << " invalid points, remaining: " 
              << clean_cloud->size() << " points" << std::endl;
    
    if (clean_cloud->empty()) {
        std::cerr << "Point cloud became empty after cleaning" << std::endl;
        return false;
    }
    
    // Apply voxel grid downsampling for large point clouds
    if (params_.enable_voxel_downsampling && 
        clean_cloud->size() > static_cast<size_t>(params_.downsampling_threshold)) {
        
        std::cout << "Large point cloud detected, applying voxel grid downsampling..." << std::endl;
        std::cout << "Voxel size: " << params_.voxel_size << " meters" << std::endl;
        
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        try {
            voxel_filter.setInputCloud(clean_cloud);
            voxel_filter.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
            voxel_filter.filter(*downsampled_cloud);
            
            if (downsampled_cloud->empty()) {
                std::cerr << "Voxel downsampling resulted in empty cloud, using original" << std::endl;
                input_cloud_ = clean_cloud;
            } else {
                input_cloud_ = downsampled_cloud;
                double reduction_ratio = 100.0 * (1.0 - static_cast<double>(input_cloud_->size()) / clean_cloud->size());
                std::cout << "Voxel downsampling: " << clean_cloud->size() << " → " << input_cloud_->size() 
                          << " points (" << reduction_ratio << "% reduction)" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Voxel downsampling failed: " << e.what() << std::endl;
            std::cerr << "Using original point cloud" << std::endl;
            input_cloud_ = clean_cloud;
        }
    } else {
        input_cloud_ = clean_cloud;
        std::cout << "Using full point cloud with " << input_cloud_->size() << " points" << std::endl;
        if (!params_.enable_voxel_downsampling) {
            std::cout << "Note: Voxel downsampling is disabled in parameters" << std::endl;
        }
    }
    
    return true;
}


bool TraversableExtractor::performRegionGrowingWithSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr& seed_points) {
    if (!input_cloud_ || input_cloud_->empty()) {
        std::cerr << "Input cloud is empty for region growing" << std::endl;
        return false;
    }
    
    if (!seed_points || seed_points->empty()) {
        std::cerr << "No seed points provided for region growing" << std::endl;
        return false;
    }
    
    std::cout << "Starting seed-guided region growing with " << input_cloud_->size() 
              << " points and " << seed_points->size() << " seeds..." << std::endl;
    
    // Step 1: Estimate normals for the input cloud
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = 
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_cloud_);
    normal_estimator.setKSearch(50);  // Use K neighbors for stable normal estimation
    
    try {
        normal_estimator.compute(*normals);
        std::cout << "Computed normals for " << normals->size() << " points" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Normal estimation failed: " << e.what() << std::endl;
        return false;
    }
    
    // Step 2: Filter ground points using normal and height constraints
    std::cout << "Filtering ground points based on surface constraints..." << std::endl;
    auto ground_result = filterGroundPoints(input_cloud_, normals, seed_points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = ground_result.first;
    std::vector<int> ground_indices = ground_result.second;
    
    if (!ground_cloud || ground_cloud->empty()) {
        std::cerr << "No ground points found after filtering" << std::endl;
        return false;
    }

    if (!params_.debug_ground_candidates_file.empty()) {
        try {
            if (Utils::savePointCloud(ground_cloud, params_.debug_ground_candidates_file)) {
                std::cout << "Saved " << ground_cloud->size()
                          << " ground candidate points to: "
                          << params_.debug_ground_candidates_file << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to save ground candidates to "
                      << params_.debug_ground_candidates_file << ": " << e.what() << std::endl;
        }
    }
    
    // Create ground normals by extracting normals at ground indices
    pcl::PointCloud<pcl::Normal>::Ptr ground_normals(new pcl::PointCloud<pcl::Normal>);
    ground_normals->reserve(ground_indices.size());
    for (int idx : ground_indices) {
        ground_normals->points.push_back(normals->points[idx]);
    }
    ground_normals->width = ground_normals->points.size();
    ground_normals->height = 1;
    ground_normals->is_dense = true;
    
    // Create KdTree for ground cloud
    pcl::search::Search<pcl::PointXYZI>::Ptr ground_tree = 
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    ground_tree->setInputCloud(ground_cloud);
    
    // Step 3: Find seed indices in ground cloud
    // For each seed point, find the closest point in the filtered ground cloud
    std::vector<int> seed_indices;
    seed_indices.reserve(seed_points->size());
    
    for (const auto& seed_point : seed_points->points) {
        float min_dist_sq = std::numeric_limits<float>::max();
        int closest_idx = -1;
        
        // Search for closest point in ground cloud
        for (size_t i = 0; i < ground_cloud->size(); ++i) {
            const auto& point = ground_cloud->points[i];
            float dx = point.x - seed_point.x;
            float dy = point.y - seed_point.y;
            float dz = point.z - seed_point.z;
            float dist_sq = dx*dx + dy*dy + dz*dz;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        
        if (closest_idx >= 0 && min_dist_sq < (params_.seed_search_radius * params_.seed_search_radius)) {
            seed_indices.push_back(closest_idx);
        }
    }
    
    std::cout << "Found " << seed_indices.size() << " valid seed indices" << std::endl;
    
    if (seed_indices.empty()) {
        std::cerr << "No valid seed indices found" << std::endl;
        return false;
    }
    
    // Step 4: Setup region growing on filtered ground points
    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(params_.region_params.min_cluster_size);
    reg.setMaxClusterSize(params_.region_params.max_cluster_size);
    reg.setSearchMethod(ground_tree);
    reg.setNumberOfNeighbours(30);  // Number of neighbors for region growing
    reg.setInputCloud(ground_cloud);
    reg.setInputNormals(ground_normals);
    
    // Convert angle threshold from degrees to radians
    float smoothness_threshold = params_.region_params.normal_angle_threshold * M_PI / 180.0f;
    reg.setSmoothnessThreshold(smoothness_threshold);
    reg.setCurvatureThreshold(params_.region_params.curvature_threshold);
    
    // Step 4: Perform region growing
    std::vector<pcl::PointIndices> clusters;
    
    try {
        reg.extract(clusters);
        std::cout << "Region growing found " << clusters.size() << " clusters" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Region growing failed: " << e.what() << std::endl;
        return false;
    }
    
    if (clusters.empty()) {
        std::cout << "No clusters found by region growing" << std::endl;
        return false;
    }
    
    // Step 5: Filter clusters that contain seed points
    regions_.clear();
    std::vector<pcl::PointIndices> valid_clusters;
    
    for (const auto& cluster : clusters) {
        // Check if this cluster contains any of our seed points
        bool contains_seed = false;
        for (int seed_idx : seed_indices) {
            // Check if seed_idx is in this cluster
            if (std::find(cluster.indices.begin(), cluster.indices.end(), seed_idx) != cluster.indices.end()) {
                contains_seed = true;
                break;
            }
        }
        
        bool seed_condition = !params_.require_seed_containment || contains_seed;

        if (seed_condition && cluster.indices.size() >= static_cast<size_t>(params_.min_region_size)) {
            valid_clusters.push_back(cluster);
        }
    }
    
    std::cout << "Found " << valid_clusters.size() << " valid clusters"
              << (params_.require_seed_containment ? " containing seed points" : " after region-size filtering")
              << std::endl;
    
    // Step 6: Convert valid clusters to point clouds
    int region_count = 0;
    for (size_t i = 0; i < valid_clusters.size(); ++i) {
        const pcl::PointIndices& cluster = valid_clusters[i];
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_cluster->points.reserve(cluster.indices.size());
        
        for (const int& idx : cluster.indices) {
            cloud_cluster->points.push_back(ground_cloud->points[idx]);
        }
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        regions_.push_back(cloud_cluster);
        region_count++;
        
        std::cout << "Region " << region_count << ": " << cloud_cluster->size() << " points" << std::endl;
        
        // Save individual cluster for debugging (optional)
        // std::string cluster_file = "output/region_" + std::to_string(region_count) + ".pcd";
        // if (Utils::savePointCloud(cloud_cluster, cluster_file)) {
        //     std::cout << "Saved region " << region_count << " to: " << cluster_file << std::endl;
        // }
    }
    
    // Step 7: Combine all valid regions
    if (!regions_.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (const auto& region : regions_) {
            *combined_cloud += *region;
        }
        
        traversable_cloud_ = combined_cloud;
        
        std::cout << "Combined " << regions_.size() << " regions into traversable areas with " 
                  << combined_cloud->size() << " points" << std::endl;
        
        return true;
    }
    
    std::cerr << "No valid regions found" << std::endl;
    return false;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<int>> TraversableExtractor::filterGroundPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& seed_points) {

    if (!cloud || !normals || cloud->size() != normals->size()) {
        std::cerr << "Invalid input for ground point filtering" << std::endl;
        return std::make_pair(pcl::PointCloud<pcl::PointXYZI>::Ptr(), std::vector<int>());
    }

    if (!seed_points || seed_points->empty()) {
        std::cerr << "No seed points provided for ground filtering" << std::endl;
        return std::make_pair(pcl::PointCloud<pcl::PointXYZI>::Ptr(), std::vector<int>());
    }

    std::cout << "Filtering ground points using " << seed_points->size() << " seed points" << std::endl;
    std::cout << "  Height filter: " << (params_.enable_height_filter ? "ENABLED" : "DISABLED") << std::endl;

    if (params_.enable_height_filter) {
        std::cout << "  XY search radius: " << params_.seed_xy_search_radius << " m" << std::endl;
        std::cout << "  Height diff threshold: " << params_.seed_height_diff_threshold << " m" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> ground_indices;

    ground_cloud->reserve(cloud->size() / 2); // Estimate
    ground_indices.reserve(cloud->size() / 2);

    int ground_count = 0;
    int no_nearby_seed_count = 0;
    int height_rejected_count = 0;

    if (params_.enable_height_filter) {
        // ========== MODE 1: Height filtering ENABLED ==========
        // Use XY plane search + height-closest seed selection

        // Build KD-tree for seed points (for efficient XY radius search)
        pcl::search::KdTree<pcl::PointXYZI> seed_kdtree;
        seed_kdtree.setInputCloud(seed_points);

        for (size_t i = 0; i < cloud->size(); ++i) {
            const pcl::PointXYZI& point = cloud->points[i];
            const pcl::Normal& normal = normals->points[i];

            // Step 1: Find all seed points within XY search radius
            std::vector<int> candidate_indices;
            std::vector<float> candidate_distances;

            // Use radius search to find nearby seeds in 3D space
            // We'll filter by XY distance later
            if (seed_kdtree.radiusSearch(point, params_.seed_xy_search_radius * 1.5,
                                          candidate_indices, candidate_distances) > 0) {

                // Step 2: Filter candidates by XY distance and find height-closest seed
                float min_height_diff = std::numeric_limits<float>::max();
                int best_seed_idx = -1;

                for (size_t j = 0; j < candidate_indices.size(); ++j) {
                    const pcl::PointXYZI& seed = seed_points->points[candidate_indices[j]];

                    // Calculate XY distance (ignore Z)
                    float dx = point.x - seed.x;
                    float dy = point.y - seed.y;
                    float xy_distance = std::sqrt(dx * dx + dy * dy);

                    // Only consider seeds within XY search radius
                    if (xy_distance <= params_.seed_xy_search_radius) {
                        float height_diff = std::abs(point.z - seed.z);

                        // Find the seed with minimum height difference
                        if (height_diff < min_height_diff) {
                            min_height_diff = height_diff;
                            best_seed_idx = candidate_indices[j];
                        }
                    }
                }

                // Step 3: Check if we found a valid seed with acceptable height difference
                if (best_seed_idx >= 0) {
                    if (min_height_diff <= params_.seed_height_diff_threshold) {
                        // Use the height-closest seed for ground point validation
                        float seed_height = seed_points->points[best_seed_idx].z;

                        if (isGroundPoint(point, normal, seed_height)) {
                            ground_cloud->points.push_back(point);
                            ground_indices.push_back(static_cast<int>(i));
                            ground_count++;
                        }
                    } else {
                        height_rejected_count++;
                    }
                } else {
                    no_nearby_seed_count++;
                }
            } else {
                no_nearby_seed_count++;
            }
        }
    } else {
        // ========== MODE 2: Height filtering DISABLED ==========
        // Only check normal direction, completely ignore height constraints

        std::cout << "  Skipping height filtering - using normal direction only" << std::endl;

        for (size_t i = 0; i < cloud->size(); ++i) {
            const pcl::PointXYZI& point = cloud->points[i];
            const pcl::Normal& normal = normals->points[i];

            // Only check normal direction (same logic as in isGroundPoint but without height check)
            if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z)) {
                continue;
            }

            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }

            // Check if normal is pointing upward (close to vertical)
            Eigen::Vector3f up_vector(0, 0, 1);
            Eigen::Vector3f point_normal(normal.normal_x, normal.normal_y, normal.normal_z);
            point_normal.normalize();

            float dot_product = up_vector.dot(point_normal);
            dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
            float angle_to_vertical = std::acos(std::abs(dot_product)) * 180.0f / M_PI;

            if (angle_to_vertical <= params_.max_ground_normal_angle) {
                // Accept point based on normal direction alone
                ground_cloud->points.push_back(point);
                ground_indices.push_back(static_cast<int>(i));
                ground_count++;
            }
        }
    }

    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    std::cout << "Filtered " << ground_count << " ground points from " << cloud->size()
              << " total points (" << (100.0 * ground_count / cloud->size()) << "%)" << std::endl;

    if (params_.enable_height_filter) {
        std::cout << "  Points with no nearby seed: " << no_nearby_seed_count << std::endl;
        std::cout << "  Points rejected by height: " << height_rejected_count << std::endl;
    }

    return std::make_pair(ground_cloud, ground_indices);
}

bool TraversableExtractor::isGroundPoint(const pcl::PointXYZI& point, const pcl::Normal& normal, float seed_height) {
    // Check if normal vectors are valid (not NaN)
    if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z)) {
        return false;
    }

    // Check if point is valid
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return false;
    }

    // 1. Calculate angle between normal and vertical direction (up vector)
    Eigen::Vector3f up_vector(0, 0, 1);
    Eigen::Vector3f point_normal(normal.normal_x, normal.normal_y, normal.normal_z);

    // Ensure normal is unit vector
    point_normal.normalize();

    // Calculate angle between normal and up vector
    float dot_product = up_vector.dot(point_normal);

    // Clamp dot product to [-1, 1] to handle numerical errors
    dot_product = std::max(-1.0f, std::min(1.0f, dot_product));

    // Calculate angle (in degrees) - this is the absolute angle regardless of direction
    float angle_to_vertical = std::acos(std::abs(dot_product)) * 180.0f / M_PI;

    // If angle is too large, the surface is not close to vertical direction
    if (angle_to_vertical > params_.max_ground_normal_angle) {
        return false; // Normal not pointing close enough to vertical
    }

    // 2. Check height constraint relative to current seed point
    float height_diff = std::abs(point.z - seed_height);
    if (height_diff > params_.seed_height_diff_threshold) {
        return false; // Too far from seed height
    }

    return true; // Point satisfies all ground constraints
}

} // namespace map_process
