#include <ros/ros.h>
#include <iostream>
#include <string>
#include "map_process/traversable_extractor.h"
#include "map_process/utils.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_process_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std::cout << "=== Map Process Node Started ===" << std::endl;
    
    // Parameters
    std::string point_cloud_file;
    std::string trajectory_file;
    std::string output_file;
    
    // Get parameters
    private_nh.param<std::string>("point_cloud_file", point_cloud_file, 
                                  "data/scans.pcd");
    private_nh.param<std::string>("trajectory_file", trajectory_file, 
                                  "data/fe_lio_pose.txt");
    private_nh.param<std::string>("output_file", output_file, 
                                  "output/traversable_areas.pcd");

    size_t output_sep = output_file.find_last_of("/");
    std::string output_dir = output_sep == std::string::npos ? "" : output_file.substr(0, output_sep);
    if (!output_dir.empty()) {
        std::string mkdir_cmd = "mkdir -p " + output_dir;
        system(mkdir_cmd.c_str());
    }
    
    // Create extraction parameters
    map_process::ExtractionParams params;
    
    // Load parameters from ROS parameter server
    private_nh.param("trajectory_sample_interval", params.trajectory_sample_interval, 1.0);
    private_nh.param("seed_search_radius", params.seed_search_radius, 0.5);
    private_nh.param("seed_height_offset", params.seed_height_offset, 1.0);
    private_nh.param("merge_distance_threshold", params.merge_distance_threshold, 0.2);
    private_nh.param("min_region_size", params.min_region_size, 100);
    private_nh.param("remove_outliers", params.remove_outliers, true);
    
    // Region growing parameters
    private_nh.param("normal_angle_threshold", params.region_params.normal_angle_threshold, 30.0);
    private_nh.param("curvature_threshold", params.region_params.curvature_threshold, 1.0);
    private_nh.param("height_threshold", params.region_params.height_threshold, 0.1);
    private_nh.param("search_radius", params.region_params.search_radius, 0.3);
    private_nh.param("min_cluster_size", params.region_params.min_cluster_size, 50);
    private_nh.param("max_cluster_size", params.region_params.max_cluster_size, 1000000);
    private_nh.param("normal_search_radius", params.region_params.normal_search_radius, 0.2);
    
    // Ground surface constraint parameters
    private_nh.param("max_ground_normal_angle", params.max_ground_normal_angle, 25.0);
    private_nh.param("enable_height_filter", params.enable_height_filter, true);
    private_nh.param("seed_height_diff_threshold", params.seed_height_diff_threshold, 0.1);
    private_nh.param("seed_xy_search_radius", params.seed_xy_search_radius, 5.0);
    
    // Point cloud downsampling parameters
    private_nh.param("enable_voxel_downsampling", params.enable_voxel_downsampling, true);
    private_nh.param("voxel_size", params.voxel_size, 0.05);
    private_nh.param("downsampling_threshold", params.downsampling_threshold, 1000000);
    
    // Trajectory format parameter
    private_nh.param("trajectory_format", params.trajectory_format, 1);
    private_nh.param("require_seed_containment", params.require_seed_containment, true);

    private_nh.param<std::string>("debug_seed_points_file", params.debug_seed_points_file,
                                  output_dir.empty() ? "seed_points.pcd" : output_dir + "/seed_points.pcd");
    private_nh.param<std::string>("debug_ground_candidates_file", params.debug_ground_candidates_file,
                                  output_dir.empty() ? "ground_candidates.pcd" : output_dir + "/ground_candidates.pcd");
    
    // Print parameters
    std::cout << "\\nParameters:" << std::endl;
    std::cout << "  Point cloud file: " << point_cloud_file << std::endl;
    std::cout << "  Trajectory file: " << trajectory_file << std::endl;
    std::cout << "  Output file: " << output_file << std::endl;
    std::cout << "  Trajectory sample interval: " << params.trajectory_sample_interval << " m" << std::endl;
    std::cout << "  Seed search radius: " << params.seed_search_radius << " m" << std::endl;
    std::cout << "  Seed height offset: " << params.seed_height_offset << " m" << std::endl;
    std::cout << "  Normal angle threshold: " << params.region_params.normal_angle_threshold << " deg" << std::endl;
    std::cout << "  Height threshold: " << params.region_params.height_threshold << " m" << std::endl;
    std::cout << "  Search radius: " << params.region_params.search_radius << " m" << std::endl;
    std::cout << "  Min cluster size: " << params.region_params.min_cluster_size << std::endl;
    std::cout << "  Max ground normal angle: " << params.max_ground_normal_angle << " deg" << std::endl;
    std::cout << "  Height filter: " << (params.enable_height_filter ? "enabled" : "disabled") << std::endl;
    std::cout << "  Seed height diff threshold: " << params.seed_height_diff_threshold << " m" << std::endl;
    std::cout << "  Seed XY search radius: " << params.seed_xy_search_radius << " m" << std::endl;
    std::cout << "  Voxel downsampling: " << (params.enable_voxel_downsampling ? "enabled" : "disabled") << std::endl;
    std::cout << "  Voxel size: " << params.voxel_size << " m" << std::endl;
    std::cout << "  Downsampling threshold: " << params.downsampling_threshold << " points" << std::endl;
    std::cout << "  Trajectory format: " << params.trajectory_format << std::endl;
    std::cout << "  Require seed containment: " << (params.require_seed_containment ? "true" : "false") << std::endl;
    std::cout << "  Debug seed points file: " << params.debug_seed_points_file << std::endl;
    std::cout << "  Debug ground candidates file: " << params.debug_ground_candidates_file << std::endl;
    
    // Create traversable extractor
    map_process::TraversableExtractor extractor;
    extractor.setParameters(params);
    
    // Load data
    std::cout << "\\n=== Loading Data ===" << std::endl;
    
    if (!extractor.loadPointCloud(point_cloud_file)) {
        ROS_ERROR("Failed to load point cloud from: %s", point_cloud_file.c_str());
        return -1;
    }
    
    if (!extractor.loadTrajectory(trajectory_file)) {
        ROS_ERROR("Failed to load trajectory from: %s", trajectory_file.c_str());
        return -1;
    }
    
    // Extract traversable areas
    std::cout << "\\n=== Processing ===" << std::endl;
    
    ros::Time start_time = ros::Time::now();
    
    if (!extractor.extractTraversableAreas()) {
        ROS_ERROR("Failed to extract traversable areas");
        return -1;
    }
    
    ros::Duration processing_time = ros::Time::now() - start_time;
    
    // Get statistics
    int total_points, traversable_points, num_regions;
    extractor.getStatistics(total_points, traversable_points, num_regions);
    
    std::cout << "\\n=== Results ===" << std::endl;
    std::cout << "  Processing time: " << processing_time.toSec() << " seconds" << std::endl;
    std::cout << "  Total input points: " << total_points << std::endl;
    std::cout << "  Traversable points: " << traversable_points << std::endl;
    std::cout << "  Coverage: " << (100.0 * traversable_points / total_points) << "%" << std::endl;
    std::cout << "  Number of regions: " << num_regions << std::endl;
    
    // Save results
    std::cout << "\\n=== Saving Results ===" << std::endl;
    
    // Create output directory if it doesn't exist
    if (!output_dir.empty()) {
        std::string mkdir_cmd = "mkdir -p " + output_dir;
        system(mkdir_cmd.c_str());
    }
    
    if (!extractor.saveTraversableAreas(output_file)) {
        ROS_ERROR("Failed to save traversable areas to: %s", output_file.c_str());
        return -1;
    }
    
    // Save individual regions (optional)
    bool save_individual_regions = false;
    private_nh.param("save_individual_regions", save_individual_regions, false);
    
    if (save_individual_regions) {
        std::cout << "Saving individual regions..." << std::endl;
        auto regions = extractor.getRegions();
        
        for (size_t i = 0; i < regions.size(); ++i) {
            std::string region_file = output_dir + "/region_" + std::to_string(i) + ".pcd";
            map_process::Utils::savePointCloud(regions[i], region_file);
        }
    }
    
    std::cout << "\\n=== Processing Complete ===" << std::endl;
    std::cout << "Results saved to: " << output_file << std::endl;
    
    return 0;
}
