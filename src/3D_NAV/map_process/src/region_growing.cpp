#include "map_process/region_growing.h"
#include "map_process/utils.h"
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <algorithm>

namespace map_process {

TraversableRegionGrowing::TraversableRegionGrowing() 
    : input_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
    , seed_points_(new pcl::PointCloud<pcl::PointXYZI>)
    , normals_(new pcl::PointCloud<pcl::Normal>)
    , kdtree_(new pcl::search::KdTree<pcl::PointXYZI>) {
}

TraversableRegionGrowing::~TraversableRegionGrowing() {
}

void TraversableRegionGrowing::setParameters(const RegionGrowingParams& params) {
    params_ = params;
}

void TraversableRegionGrowing::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    input_cloud_ = cloud;
    if (cloud && !cloud->empty()) {
        try {
            kdtree_->setInputCloud(cloud);
        } catch (const std::exception& e) {
            std::cerr << "Failed to set input cloud for KdTree: " << e.what() << std::endl;
        }
    }
}

void TraversableRegionGrowing::setSeedPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& seeds) {
    seed_points_ = seeds;
}

bool TraversableRegionGrowing::extract(std::vector<std::vector<int>>& clusters) {
    clusters.clear();
    
    if (!input_cloud_ || input_cloud_->empty()) {
        std::cerr << "Input cloud is empty" << std::endl;
        return false;
    }
    
    if (!seed_points_ || seed_points_->empty()) {
        std::cerr << "No seed points provided" << std::endl;
        return false;
    }
    
    // Estimate normals for the input cloud
    if (!estimateNormals()) {
        std::cerr << "Failed to estimate normals" << std::endl;
        return false;
    }
    
    std::cout << "Starting region growing with " << seed_points_->size() 
              << " seed points..." << std::endl;
    
    std::unordered_set<int> global_visited;
    
    // Process each seed point
    for (size_t seed_idx = 0; seed_idx < seed_points_->size(); ++seed_idx) {
        // Find closest point in input cloud to this seed
        int closest_idx = findClosestPoint(seed_points_->points[seed_idx]);
        
        if (closest_idx < 0 || global_visited.count(closest_idx)) {
            continue; // Skip if not found or already processed
        }
        
        // Check if this point is potentially traversable
        if (!isTraversable(closest_idx)) {
            continue;
        }
        
        std::vector<int> cluster;
        int points_added = growRegion(closest_idx, global_visited, cluster);
        
        // Only keep clusters that meet size requirements
        if (points_added >= params_.min_cluster_size && 
            points_added <= params_.max_cluster_size) {
            clusters.push_back(cluster);
            std::cout << "Generated cluster " << clusters.size() 
                      << " with " << points_added << " points" << std::endl;
        }
    }
    
    std::cout << "Region growing completed. Generated " << clusters.size() 
              << " valid clusters" << std::endl;
    
    return !clusters.empty();
}

bool TraversableRegionGrowing::estimateNormals() {
    if (!input_cloud_ || input_cloud_->empty()) {
        return false;
    }
    
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(input_cloud_);
    normal_estimator.setSearchMethod(kdtree_);
    normal_estimator.setRadiusSearch(params_.normal_search_radius);
    
    normal_estimator.compute(*normals_);
    
    std::cout << "Estimated normals for " << normals_->size() << " points" << std::endl;
    
    return normals_->size() == input_cloud_->size();
}

bool TraversableRegionGrowing::canMerge(int idx1, int idx2) const {
    if (idx1 < 0 || idx1 >= static_cast<int>(input_cloud_->size()) ||
        idx2 < 0 || idx2 >= static_cast<int>(input_cloud_->size()) ||
        idx1 >= static_cast<int>(normals_->size()) ||
        idx2 >= static_cast<int>(normals_->size())) {
        return false;
    }
    
    const pcl::PointXYZI& p1 = input_cloud_->points[idx1];
    const pcl::PointXYZI& p2 = input_cloud_->points[idx2];
    const pcl::Normal& n1 = normals_->points[idx1];
    const pcl::Normal& n2 = normals_->points[idx2];
    
    // Check if normals are valid
    if (!std::isfinite(n1.normal_x) || !std::isfinite(n1.normal_y) || !std::isfinite(n1.normal_z) ||
        !std::isfinite(n2.normal_x) || !std::isfinite(n2.normal_y) || !std::isfinite(n2.normal_z)) {
        return false;
    }
    
    // 1. Normal angle constraint
    Eigen::Vector3f normal1(n1.normal_x, n1.normal_y, n1.normal_z);
    Eigen::Vector3f normal2(n2.normal_x, n2.normal_y, n2.normal_z);
    
    double angle = Utils::angleBetweenVectors(normal1, normal2);
    double angle_threshold = Utils::degToRad(params_.normal_angle_threshold);
    
    if (angle > angle_threshold) {
        return false;
    }
    
    // 2. Curvature constraint
    if (std::abs(n1.curvature - n2.curvature) > params_.curvature_threshold) {
        return false;
    }
    
    // 3. Height constraint
    if (std::abs(p1.z - p2.z) > params_.height_threshold) {
        return false;
    }
    
    return true;
}

int TraversableRegionGrowing::findClosestPoint(const pcl::PointXYZI& seed_point) const {
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    
    if (kdtree_->nearestKSearch(seed_point, 1, indices, distances) > 0) {
        return indices[0];
    }
    
    return -1;
}

int TraversableRegionGrowing::growRegion(int seed_idx, std::unordered_set<int>& visited, std::vector<int>& cluster) {
    std::queue<int> points_to_process;
    points_to_process.push(seed_idx);
    visited.insert(seed_idx);
    cluster.push_back(seed_idx);
    
    int points_added = 1;
    
    while (!points_to_process.empty() && points_added < params_.max_cluster_size) {
        int current_idx = points_to_process.front();
        points_to_process.pop();
        
        // Find neighbors within search radius
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        
        int neighbors_found = kdtree_->radiusSearch(
            input_cloud_->points[current_idx],
            params_.search_radius,
            neighbor_indices,
            neighbor_distances
        );
        
        for (int i = 0; i < neighbors_found; ++i) {
            int neighbor_idx = neighbor_indices[i];
            
            // Skip if already visited
            if (visited.count(neighbor_idx)) {
                continue;
            }
            
            // Check if neighbor can be merged with current point
            if (canMerge(current_idx, neighbor_idx) && isTraversable(neighbor_idx)) {
                visited.insert(neighbor_idx);
                cluster.push_back(neighbor_idx);
                points_to_process.push(neighbor_idx);
                points_added++;
                
                if (points_added >= params_.max_cluster_size) {
                    break;
                }
            }
        }
    }
    
    return points_added;
}

bool TraversableRegionGrowing::isTraversable(int idx) const {
    if (idx < 0 || idx >= static_cast<int>(normals_->size())) {
        return false;
    }
    
    const pcl::Normal& normal = normals_->points[idx];
    
    // Check if normal is valid
    if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z)) {
        return false;
    }
    
    // Check if surface is roughly horizontal (normal pointing upward)
    Eigen::Vector3f n(normal.normal_x, normal.normal_y, normal.normal_z);
    Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
    
    double angle_with_vertical = Utils::angleBetweenVectors(n, up);
    double max_slope_angle = Utils::degToRad(45.0); // 45 degrees max slope
    
    if (angle_with_vertical > max_slope_angle) {
        return false;
    }
    
    // Check curvature (lower curvature indicates flatter surface)
    if (normal.curvature > 0.1) { // Threshold for surface smoothness
        return false;
    }
    
    return true;
}

} // namespace map_process
