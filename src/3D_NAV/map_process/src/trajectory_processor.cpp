#include "map_process/trajectory_processor.h"
#include "map_process/utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace map_process {

TrajectoryProcessor::TrajectoryProcessor() : trajectory_format_(1) {
}

TrajectoryProcessor::~TrajectoryProcessor() {
}

void TrajectoryProcessor::setTrajectoryFormat(int format) {
    if (format >= 1 && format <= 3) {
        trajectory_format_ = format;
        std::cout << "Set trajectory format to: " << format << std::endl;
    } else {
        std::cerr << "Invalid trajectory format: " << format << ". Using format 1." << std::endl;
        trajectory_format_ = 1;
    }
}

bool TrajectoryProcessor::loadTrajectory(const std::string& filename) {
    if (!Utils::fileExists(filename)) {
        std::cerr << "Trajectory file does not exist: " << filename << std::endl;
        return false;
    }
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open trajectory file: " << filename << std::endl;
        return false;
    }
    
    trajectory_.clear();
    std::string line;
    int line_count = 0;
    
    while (std::getline(file, line)) {
        line_count++;
        line = Utils::trim(line);
        
        // Skip empty lines
        if (line.empty()) {
            continue;
        }
        
        RobotPose pose;
        if (parsePoseLine(line, pose)) {
            trajectory_.push_back(pose);
        } else {
            std::cerr << "Failed to parse line " << line_count << ": " << line << std::endl;
        }
    }
    
    file.close();
    
    if (trajectory_.empty()) {
        std::cerr << "No valid poses loaded from trajectory file" << std::endl;
        return false;
    }
    
    std::cout << "Loaded " << trajectory_.size() << " poses from trajectory file" << std::endl;
    std::cout << "Trajectory length: " << getTrajectoryLength() << " meters" << std::endl;
    
    return true;
}

std::vector<RobotPose> TrajectoryProcessor::sampleTrajectory(double interval) {
    std::vector<RobotPose> sampled_poses;
    
    if (trajectory_.empty()) {
        std::cerr << "No trajectory loaded for sampling" << std::endl;
        return sampled_poses;
    }
    
    if (trajectory_.size() == 1) {
        sampled_poses.push_back(trajectory_[0]);
        return sampled_poses;
    }
    
    // Always include the first pose
    sampled_poses.push_back(trajectory_[0]);
    double accumulated_distance = 0.0;
    
    for (size_t i = 1; i < trajectory_.size(); ++i) {
        double distance = calculateDistance(trajectory_[i-1], trajectory_[i]);
        accumulated_distance += distance;
        
        if (accumulated_distance >= interval) {
            sampled_poses.push_back(trajectory_[i]);
            accumulated_distance = 0.0;
        }
    }
    
    // Always include the last pose if it's not already included
    // Check if last pose is different by comparing positions (since timestamps might be 0)
    const auto& last_sampled = sampled_poses.back();
    const auto& last_trajectory = trajectory_.back();
    double distance_to_last = calculateDistance(last_sampled, last_trajectory);
    
    if (distance_to_last > 1e-6) {  // Use small epsilon for floating point comparison
        sampled_poses.push_back(trajectory_.back());
    }
    
    std::cout << "Sampled " << sampled_poses.size() << " poses from " 
              << trajectory_.size() << " total poses (interval: " << interval << "m)" << std::endl;
    
    return sampled_poses;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TrajectoryProcessor::findSeedPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<RobotPose>& sampled_poses,
    double search_radius,
    double height_offset) {
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::cout << "Debug: findSeedPoints called with cloud size: " 
              << (cloud ? cloud->size() : 0) << std::endl;
    
    if (!cloud || cloud->empty()) {
        std::cerr << "Input point cloud is empty" << std::endl;
        return seed_points;
    }
    
    if (sampled_poses.empty()) {
        std::cerr << "No sampled poses provided" << std::endl;
        return seed_points;
    }
    
    // Skip KdTree due to PCL version conflicts
    std::cout << "Debug: Using manual search for " << cloud->size() << " points" << std::endl;
    
    int total_candidates = 0;
    int valid_seeds = 0;
    
    for (const auto& pose : sampled_poses) {
        // Define search region around the robot position
        double robot_x = pose.position.x();
        double robot_y = pose.position.y();
        double robot_z = pose.position.z();
        
        // Look for points below the robot (ground points)
        double min_z = robot_z - height_offset - 0.5;  // Extra margin
        double max_z = robot_z - height_offset + 0.5;  // Extra margin
        
        std::vector<pcl::PointXYZI> candidates;
        
        for (const auto& point : cloud->points) {
            // Check if point is within horizontal search radius
            double dx = point.x - robot_x;
            double dy = point.y - robot_y;
            double horizontal_distance = std::sqrt(dx*dx + dy*dy);
            
            if (horizontal_distance <= search_radius) {
                // Check if point is at appropriate height (ground level)
                if (point.z >= min_z && point.z <= max_z) {
                    candidates.push_back(point);
                    total_candidates++;
                }
            }
        }
        
        // Select representative seed points from candidates
        if (!candidates.empty()) {
            // For now, just add the first few candidates
            int max_seeds_per_pose = std::min(3, static_cast<int>(candidates.size()));
            for (int i = 0; i < max_seeds_per_pose; ++i) {
                seed_points->points.push_back(candidates[i]);
                valid_seeds++;
            }
        }
    }
    
    seed_points->width = seed_points->points.size();
    seed_points->height = 1;
    seed_points->is_dense = true;
    
    std::cout << "Found " << total_candidates << " candidate points" << std::endl;
    std::cout << "Selected " << valid_seeds << " seed points from " 
              << sampled_poses.size() << " trajectory poses" << std::endl;
    
    return seed_points;
}

double TrajectoryProcessor::getTrajectoryLength() const {
    if (trajectory_.size() < 2) {
        return 0.0;
    }
    
    double total_length = 0.0;
    for (size_t i = 1; i < trajectory_.size(); ++i) {
        total_length += calculateDistance(trajectory_[i-1], trajectory_[i]);
    }
    
    return total_length;
}

double TrajectoryProcessor::calculateDistance(const RobotPose& pose1, const RobotPose& pose2) const {
    Eigen::Vector3d diff = pose1.position - pose2.position;
    return diff.norm();
}

bool TrajectoryProcessor::parsePoseLine(const std::string& line, RobotPose& pose) const {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    
    while (iss >> token) {
        tokens.push_back(token);
    }
    
    try {
        switch (trajectory_format_) {
            case 1: {
                // Format 1: timestamp x y z qx qy qz qw
                if (tokens.size() < 8) {
                    return false;
                }
                pose.timestamp = std::stod(tokens[0]);
                pose.position.x() = std::stod(tokens[1]);
                pose.position.y() = std::stod(tokens[2]);
                pose.position.z() = std::stod(tokens[3]);
                pose.orientation.x() = std::stod(tokens[4]);
                pose.orientation.y() = std::stod(tokens[5]);
                pose.orientation.z() = std::stod(tokens[6]);
                pose.orientation.w() = std::stod(tokens[7]);
                
                // Normalize quaternion
                pose.orientation.normalize();
                return true;
            }
            
            case 2: {
                // Format 2: x y z param1 param2 param3
                if (tokens.size() < 6) {
                    return false;
                }
                pose.timestamp = 0.0; // No timestamp in this format
                pose.position.x() = std::stod(tokens[0]);
                pose.position.y() = std::stod(tokens[1]);
                pose.position.z() = std::stod(tokens[2]);
                
                // Set default orientation (identity quaternion)
                pose.orientation = Eigen::Quaterniond::Identity();
                return true;
            }
            
            case 3: {
                // Format 3: multiple_values where position is at columns 5-7 (indices 4-6)
                if (tokens.size() < 7) {
                    return false;
                }
                pose.timestamp = 0.0; // No timestamp in this format
                pose.position.x() = std::stod(tokens[4]); // Column 5
                pose.position.y() = std::stod(tokens[5]); // Column 6
                pose.position.z() = std::stod(tokens[6]); // Column 7
                
                // Set default orientation (identity quaternion)
                pose.orientation = Eigen::Quaterniond::Identity();
                return true;
            }
            
            default:
                std::cerr << "Unknown trajectory format: " << trajectory_format_ << std::endl;
                return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse trajectory line: " << e.what() << std::endl;
        return false;
    }
}

} // namespace map_process
