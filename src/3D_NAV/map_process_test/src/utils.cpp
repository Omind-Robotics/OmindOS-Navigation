#include "map_process/utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <exception>
#include <sys/stat.h>
#include <sys/types.h>

namespace map_process {

double Utils::degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

double Utils::radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

double Utils::angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    // Normalize vectors
    Eigen::Vector3f n1 = v1.normalized();
    Eigen::Vector3f n2 = v2.normalized();
    
    // Calculate dot product
    float dot_product = n1.dot(n2);
    
    // Clamp to avoid numerical errors
    dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
    
    return std::acos(dot_product);
}

double Utils::euclideanDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Utils::voxelFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    float leaf_size) {
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (!input || input->empty()) {
        std::cerr << "Input cloud is empty for voxel filtering" << std::endl;
        return filtered;
    }
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*filtered);
    
    std::cout << "Voxel filter: " << input->size() << " -> " << filtered->size() 
              << " points (leaf size: " << leaf_size << ")" << std::endl;
    
    return filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Utils::statisticalOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    int mean_k,
    double std_dev_mul) {
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (!input || input->empty()) {
        std::cerr << "Input cloud is empty for outlier removal" << std::endl;
        return filtered;
    }
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outlier_filter;
    outlier_filter.setInputCloud(input);
    outlier_filter.setMeanK(mean_k);
    outlier_filter.setStddevMulThresh(std_dev_mul);
    outlier_filter.filter(*filtered);
    
    std::cout << "Statistical outlier removal: " << input->size() << " -> " 
              << filtered->size() << " points" << std::endl;
    
    return filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Utils::loadPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (!fileExists(filename)) {
        std::cerr << "Point cloud file does not exist: " << filename << std::endl;
        return cloud;
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1) {
        std::cerr << "Failed to load point cloud: " << filename << std::endl;
        cloud->clear();
        return cloud;
    }
    
    std::cout << "Loaded point cloud with " << cloud->size() << " points from: " 
              << filename << std::endl;
    
    return cloud;
}

bool Utils::savePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& filename,
    bool binary) {
    
    if (!cloud || cloud->empty()) {
        std::cerr << "Cannot save empty point cloud" << std::endl;
        return false;
    }
    
    const std::string output_dir = getParentDirectory(filename);
    if (!ensureDirectoryExists(output_dir)) {
        std::cerr << "Failed to create output directory: " << output_dir << std::endl;
        return false;
    }

    try {
        int result = pcl::io::savePCDFile(filename, *cloud, binary);
        if (result == -1) {
            std::cerr << "Failed to save point cloud to: " << filename << std::endl;
            return false;
        }
    } catch (const pcl::IOException& e) {
        std::cerr << "PCL failed to save point cloud to: " << filename
                  << " (" << e.what() << ")" << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Failed to save point cloud to: " << filename
                  << " (" << e.what() << ")" << std::endl;
        return false;
    }
    
    std::cout << "Saved point cloud with " << cloud->size() << " points to: " 
              << filename << std::endl;
    
    return true;
}

void Utils::printCloudInfo(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& name) {
    
    if (!cloud) {
        std::cout << name << ": null pointer" << std::endl;
        return;
    }
    
    if (cloud->empty()) {
        std::cout << name << ": empty cloud" << std::endl;
        return;
    }
    
    // Calculate bounding box
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    std::cout << name << ":" << std::endl;
    std::cout << "  Points: " << cloud->size() << std::endl;
    std::cout << "  Bounding box:" << std::endl;
    std::cout << "    X: [" << min_x << ", " << max_x << "]" << std::endl;
    std::cout << "    Y: [" << min_y << ", " << max_y << "]" << std::endl;
    std::cout << "    Z: [" << min_z << ", " << max_z << "]" << std::endl;
    std::cout << "  Dimensions: " << (max_x - min_x) << " x " 
              << (max_y - min_y) << " x " << (max_z - min_z) << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Utils::createColoredCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (!cloud || cloud->empty()) {
        return colored_cloud;
    }
    
    colored_cloud->width = cloud->width;
    colored_cloud->height = cloud->height;
    colored_cloud->is_dense = cloud->is_dense;
    colored_cloud->points.resize(cloud->points.size());
    
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z;
        colored_cloud->points[i].r = r;
        colored_cloud->points[i].g = g;
        colored_cloud->points[i].b = b;
    }
    
    return colored_cloud;
}

std::vector<std::string> Utils::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

std::string Utils::trim(const std::string& str) {
    const std::string whitespace = " \t\n\r";
    
    size_t first = str.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return "";
    }
    
    size_t last = str.find_last_not_of(whitespace);
    return str.substr(first, (last - first + 1));
}

bool Utils::fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

std::string Utils::getParentDirectory(const std::string& filename) {
    const size_t pos = filename.find_last_of("/\\");
    if (pos == std::string::npos) {
        return ".";
    }
    if (pos == 0) {
        return filename.substr(0, 1);
    }
    return filename.substr(0, pos);
}

bool Utils::ensureDirectoryExists(const std::string& directory) {
    if (directory.empty() || directory == ".") {
        return true;
    }

    struct stat st;
    if (stat(directory.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    std::string current;
    size_t start = 0;
    if (!directory.empty() && directory[0] == '/') {
        current = "/";
        start = 1;
    }

    while (start <= directory.size()) {
        const size_t next = directory.find('/', start);
        const std::string part = directory.substr(
            start,
            next == std::string::npos ? std::string::npos : next - start);

        if (!part.empty() && part != ".") {
            if (!current.empty() && current.back() != '/') {
                current += "/";
            }
            current += part;

            if (stat(current.c_str(), &st) != 0) {
                if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                    std::cerr << "Failed to create directory: " << current << std::endl;
                    return false;
                }
            } else if (!S_ISDIR(st.st_mode)) {
                std::cerr << "Path exists but is not a directory: " << current << std::endl;
                return false;
            }
        }

        if (next == std::string::npos) {
            break;
        }
        start = next + 1;
    }

    return true;
}

std::string Utils::getCurrentTimeString() {
    std::time_t now = std::time(0);
    std::tm* local_time = std::localtime(&now);
    
    std::stringstream ss;
    ss << (local_time->tm_year + 1900) << "-"
       << (local_time->tm_mon + 1) << "-"
       << local_time->tm_mday << "_"
       << local_time->tm_hour << "-"
       << local_time->tm_min << "-"
       << local_time->tm_sec;
    
    return ss.str();
}

} // namespace map_process
