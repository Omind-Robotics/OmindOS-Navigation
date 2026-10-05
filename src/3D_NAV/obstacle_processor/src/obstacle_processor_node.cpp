#include <tf/transform_listener.h>
#include "obstacle_processor/backward.hpp"
#include "obstacle_processor/execution_classes.h"

// Debug logging control - set to 0 to disable all debug output
#ifndef DEBUG_OBSTACLE_PROCESSOR
#define DEBUG_OBSTACLE_PROCESSOR 1
#endif

#if DEBUG_OBSTACLE_PROCESSOR
  #define DEBUG_LOG(fmt, ...) ROS_INFO(fmt, ##__VA_ARGS__)
  #define DEBUG_LOG_THROTTLE(period, fmt, ...) ROS_INFO_THROTTLE(period, fmt, ##__VA_ARGS__)
#else
  #define DEBUG_LOG(fmt, ...)
  #define DEBUG_LOG_THROTTLE(period, fmt, ...)
#endif
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>  // For ApproximateVoxelGrid
#include <pcl/common/common.h>
#include <pcl/common/io.h>  // For pcl::copyPointCloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sys/time.h>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <omp.h>
#include <atomic>  // For atomic flags to prevent callback reentry
#include <mutex>   // For mutex protection

using namespace std;
using namespace Eigen;
using namespace EXECUTION;

// Optimized obstacle expansion class
class ObstacleExpander {
private:
    static std::vector<Eigen::Vector3f> expansion_offsets_;
    static bool initialized_;
    
public:
    static void initialize(float expansion) {
        if (!initialized_) {
            expansion_offsets_ = {
                {0, 0, 0},                          // 原始点
                {expansion, 0, 0}, {-expansion, 0, 0},     // X轴扩展
                {0, expansion, 0}, {0, -expansion, 0},     // Y轴扩展
                {expansion, expansion, 0}, {-expansion, -expansion, 0},   // XY对角
                {expansion, -expansion, 0}, {-expansion, expansion, 0}
            };
            initialized_ = true;
            ROS_INFO("ObstacleExpander initialized with expansion: %.3f", expansion);
        }
    }
    
    // 批量生成扩展点，减少vector操作
    static void expandObstacle(const pcl::PointXYZ& center, 
                              std::vector<pcl::PointXYZ>& cloud_output,
                              std::vector<float>& array_output) {
        // 预分配空间避免重复分配
        size_t start_size = cloud_output.size();
        cloud_output.resize(start_size + 9);
        
        size_t array_start = array_output.size();
        array_output.resize(array_start + 27); // 9个点 * 3个坐标
        
        // 批量生成扩展点
        for (size_t i = 0; i < 9; ++i) {
            const auto& offset = expansion_offsets_[i];
            
            // 点云数据
            pcl::PointXYZ& pt = cloud_output[start_size + i];
            pt.x = center.x + offset.x();
            pt.y = center.y + offset.y();
            pt.z = center.z + offset.z();
            
            // 数组数据 
            size_t array_idx = array_start + i * 3;
            array_output[array_idx] = pt.x;
            array_output[array_idx + 1] = pt.y;
            array_output[array_idx + 2] = pt.z;
        }
    }
};

// 静态成员初始化
std::vector<Eigen::Vector3f> ObstacleExpander::expansion_offsets_;
bool ObstacleExpander::initialized_ = false;


// Safe PCL VoxelGrid downsampling function with proper memory management
pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                                                          double leaf_size) {
    // Create output cloud with proper memory alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    if (!input_cloud || input_cloud->empty()) {
        if (input_cloud) {
            output_cloud->header = input_cloud->header;
        }
        output_cloud->is_dense = false;
        return output_cloud;
    }
    
    try {
        // Create VoxelGrid filter with proper initialization
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(static_cast<float>(leaf_size), 
                                static_cast<float>(leaf_size), 
                                static_cast<float>(leaf_size));
        
        // Reserve space to avoid memory fragmentation
        output_cloud->reserve(input_cloud->size() / 4); // Conservative estimate
        
        // Apply filter with error handling
        voxel_filter.filter(*output_cloud);
        
        // Ensure proper header and properties
        output_cloud->header = input_cloud->header;
        output_cloud->is_dense = input_cloud->is_dense;
        
        DEBUG_LOG("PCL VoxelGrid downsampling: %zu -> %zu points (leaf_size: %.3f)", 
                 input_cloud->size(), output_cloud->size(), leaf_size);
        
        return output_cloud;
        
    } catch (const std::exception& e) {
        ROS_ERROR("VoxelGrid downsampling failed: %s. Falling back to original cloud.", e.what());
        // Return a copy of the original cloud to avoid memory issues
        pcl::PointCloud<pcl::PointXYZ>::Ptr fallback_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::copyPointCloud(*input_cloud, *fallback_cloud);
        return fallback_cloud;
    }
}

// Custom voxel downsampling - Safe implementation without PCL VoxelGrid memory issues
// This implementation uses std::unordered_map to manually implement voxel grid filtering
// avoiding memory corruption issues in both PCL VoxelGrid and ApproximateVoxelGrid
pcl::PointCloud<pcl::PointXYZ>::Ptr spatial_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                       double leaf_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!input_cloud || input_cloud->empty()) {
        if (input_cloud) {
            output_cloud->header = input_cloud->header;
        }
        output_cloud->is_dense = false;
        return output_cloud;
    }

    try {
        // Hash function for voxel grid key
        struct VoxelKey {
            int x, y, z;

            bool operator==(const VoxelKey& other) const {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct VoxelKeyHash {
            std::size_t operator()(const VoxelKey& k) const {
                // Efficient hash combining three integers
                return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
            }
        };

        // Map voxel key to first point in that voxel
        std::unordered_map<VoxelKey, pcl::PointXYZ, VoxelKeyHash> voxel_map;
        voxel_map.reserve(input_cloud->size() / 8); // Conservative estimate

        // Inverse of leaf size for faster computation
        const double inv_leaf_size = 1.0 / leaf_size;

        // Insert points into voxel grid (keep first point in each voxel)
        for (const auto& point : input_cloud->points) {
            // Compute voxel indices
            VoxelKey key;
            key.x = static_cast<int>(std::floor(point.x * inv_leaf_size));
            key.y = static_cast<int>(std::floor(point.y * inv_leaf_size));
            key.z = static_cast<int>(std::floor(point.z * inv_leaf_size));

            // Only insert if voxel is empty (keeps first point)
            if (voxel_map.find(key) == voxel_map.end()) {
                voxel_map[key] = point;
            }
        }

        // Extract downsampled points from hash map
        output_cloud->points.reserve(voxel_map.size());
        for (const auto& kv : voxel_map) {
            output_cloud->points.push_back(kv.second);
        }

        // Set cloud properties
        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = input_cloud->is_dense;
        output_cloud->header = input_cloud->header;

        DEBUG_LOG("Custom voxel downsampling: %zu -> %zu points (leaf_size: %.3f)",
                 input_cloud->size(), output_cloud->size(), leaf_size);

        return output_cloud;

    } catch (const std::exception& e) {
        ROS_ERROR("Custom voxel downsampling failed: %s. Returning empty cloud.", e.what());
        output_cloud->header = input_cloud->header;
        output_cloud->is_dense = false;
        return output_cloud;
    }
}

// 3D Grid Index for fast spatial queries
// Optimized for 0.4m voxel global map
struct GridKey {
  int x, y, z;
  
  GridKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
  
  bool operator==(const GridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

// Hash function for GridKey
struct GridKeyHash {
  std::size_t operator()(const GridKey& k) const {
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.z) << 2);
  }
};

// Hash function for XY pair
struct PairHash {
  std::size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};

// Height layer structure for multi-layer environments (stairs, ramps, platforms)
struct HeightLayer {
  double min_z;       // Minimum Z value in this layer
  double max_z;       // Maximum Z value in this layer
  double centroid_z;  // Centroid Z value (average)
  size_t count;       // Number of points in this layer
  std::vector<pcl::PointXYZ> samples; // Optional: sample points for visualization/verification

  HeightLayer() : min_z(0), max_z(0), centroid_z(0), count(0) {}

  HeightLayer(double min, double max, double centroid, size_t cnt)
    : min_z(min), max_z(max), centroid_z(centroid), count(cnt) {}
};

class GridIndex3D {
private:
  // Multi-layer grid: each XY cell contains multiple height layers
  std::unordered_map<std::pair<int, int>, std::vector<HeightLayer>, PairHash> xyGrid_;
  double gridSize_;
  double invGridSize_;        // Cached 1/gridSize for fast division
  double halfGridSize_;       // Cached gridSize/2 for boundary checks
  double halfGridSizeSquared_; // Cached (gridSize/2)^2 for fast distance comparison

  // Layer clustering parameters
  double layerGapThreshold_;       // Height gap to split into different layers (e.g., 0.3m)
  size_t layerMergeMinPoints_;     // Minimum points to keep a layer (filter noise)
  double layerHeightTolerance_;    // Height tolerance for matching (e.g., 0.15m)
  
public:
  GridIndex3D(double gridSize = 0.1) : gridSize_(gridSize),
                                        layerGapThreshold_(0.3),
                                        layerMergeMinPoints_(3),
                                        layerHeightTolerance_(0.15) {
    invGridSize_ = 1.0 / gridSize_;
    halfGridSize_ = gridSize_ * 0.5;
    halfGridSizeSquared_ = halfGridSize_ * halfGridSize_;
  }

  void setGridSize(double size) {
    gridSize_ = size;
    invGridSize_ = 1.0 / size;
    halfGridSize_ = size * 0.5;
    halfGridSizeSquared_ = halfGridSize_ * halfGridSize_;
  }

  void setLayerParameters(double gapThreshold, size_t minPoints, double heightTolerance) {
    layerGapThreshold_ = gapThreshold;
    layerMergeMinPoints_ = minPoints;
    layerHeightTolerance_ = heightTolerance;
  }
  
  // Fast world to grid conversion using cached inverse
  std::pair<int, int> worldToXYGrid(const pcl::PointXYZ& point) const {
    return std::make_pair(
      static_cast<int>(std::floor(point.x * invGridSize_)),
      static_cast<int>(std::floor(point.y * invGridSize_))
    );
  }
  
  // Fast distance check without sqrt - returns true if points are within same grid cell
  inline bool isWithinGridCellFast(const pcl::PointXYZ& pt1, const pcl::PointXYZ& pt2) const {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    double distanceSquared = dx*dx + dy*dy;
    return distanceSquared <= halfGridSizeSquared_;
  }
  
  // Fast XY grid boundary check without sqrt
  inline bool isWithinGridBoundary(const pcl::PointXYZ& queryPt, const pcl::PointXYZ& mapPt) const {
    return (queryPt.x >= mapPt.x - halfGridSize_ && queryPt.x <= mapPt.x + halfGridSize_) &&
           (queryPt.y >= mapPt.y - halfGridSize_ && queryPt.y <= mapPt.y + halfGridSize_);
  }
  
  void clear() {
    xyGrid_.clear();
  }
  
  void buildGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    clear();

    // Step 1: Group points by XY grid cell
    std::unordered_map<std::pair<int, int>, std::vector<pcl::PointXYZ>, PairHash> tempGrid;
    for (const auto& pt : cloud) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        std::pair<int, int> key = worldToXYGrid(pt);
        tempGrid[key].push_back(pt);
      }
    }

    // Step 2: For each XY cell, perform height-based clustering
    for (auto& entry : tempGrid) {
        auto& key = entry.first;
        auto& points = entry.second;
      if (points.empty()) continue;

      // Sort points by Z coordinate
      std::sort(points.begin(), points.end(),
                [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                  return a.z < b.z;
                });

      // Perform 1D clustering along Z axis
      std::vector<HeightLayer> layers;
      HeightLayer currentLayer;
      currentLayer.min_z = static_cast<double>(points[0].z);
      currentLayer.max_z = static_cast<double>(points[0].z);
      currentLayer.centroid_z = 0.0;
      currentLayer.count = 0;

      double sum_z = 0.0;

      for (const auto& pt : points) {
        // If height gap is larger than threshold, start a new layer
        if (static_cast<double>(pt.z) - currentLayer.max_z > layerGapThreshold_) {
          // Finalize current layer
          if (currentLayer.count > 0) {
            currentLayer.centroid_z = sum_z / currentLayer.count;
            // Only keep layers with enough points (filter noise)
            if (currentLayer.count >= layerMergeMinPoints_) {
              layers.push_back(currentLayer);
            }
          }

          // Start new layer
          currentLayer = HeightLayer();
          currentLayer.min_z = static_cast<double>(pt.z);
          currentLayer.max_z = static_cast<double>(pt.z);
          currentLayer.count = 0;
          sum_z = 0.0;
        }

        // Add point to current layer
        currentLayer.max_z = std::max(currentLayer.max_z, static_cast<double>(pt.z));
        currentLayer.min_z = std::min(currentLayer.min_z, static_cast<double>(pt.z));
        currentLayer.count++;
        sum_z += static_cast<double>(pt.z);
      }

      // Don't forget the last layer
      if (currentLayer.count > 0) {
        currentLayer.centroid_z = sum_z / currentLayer.count;
        if (currentLayer.count >= layerMergeMinPoints_) {
          layers.push_back(currentLayer);
        }
      }

      // Store layers for this XY grid cell
      if (!layers.empty()) {
        xyGrid_[key] = layers;
      }
    }
  }
  
  // Get height layers for a given query point (multi-layer support)
  std::vector<HeightLayer> getLayersInGridCell(const pcl::PointXYZ& queryPoint) const {
    std::vector<HeightLayer> result;
    std::pair<int, int> centerKey = worldToXYGrid(queryPoint);

    auto it = xyGrid_.find(centerKey);
    if (it != xyGrid_.end()) {
      result = it->second; // Return all layers in this XY grid cell
    }

    return result;
  }

  // Legacy compatibility: Get all points in grid cell (flattened from all layers)
  // NOTE: This is kept for backward compatibility but multi-layer code should use getLayersInGridCell
  std::vector<pcl::PointXYZ> getPointsInGridCell(const pcl::PointXYZ& queryPoint) const {
    std::vector<pcl::PointXYZ> result;
    std::pair<int, int> centerKey = worldToXYGrid(queryPoint);

    // For backward compatibility, we need to reconstruct points from layers
    // This is a placeholder - actual implementation depends on whether we store samples
    auto it = xyGrid_.find(centerKey);
    if (it != xyGrid_.end()) {
      // Return representative points from each layer (using centroid)
      for (const auto& layer : it->second) {
        pcl::PointXYZ pt;
        // Use the query point's XY with the layer's centroid Z
        pt.x = queryPoint.x;
        pt.y = queryPoint.y;
        pt.z = layer.centroid_z;
        result.push_back(pt);
      }
    }

    return result;
  }
  
  size_t size() const {
    size_t total = 0;
    for (const auto& pair : xyGrid_) {
      // Count total points across all layers in each grid cell
      for (const auto& layer : pair.second) {
        total += layer.count;
      }
    }
    return total;
  }
  
  bool empty() const {
    return xyGrid_.empty();
  }
};

namespace backward
{
  backward::SignalHandling sh;
}

World *world = NULL;
// 全局重用的local_world对象，避免每次回调重新分配
World *local_world_reusable = NULL;
bool local_world_initialized = false;

ros::Subscriber pt_sub;
ros::Subscriber world_sub;
ros::Publisher obs_pub;
ros::Publisher obs_cost_pub;
ros::Publisher obs_array_pub;

double resolution, leaf_size, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;
string map_frame_id;
string lidar_frame_id;
string base_frame_id;

// Spatial downsampling configuration
// Note: Custom spatial downsampling provides VoxelGrid-like functionality without PCL dependencies

// Performance optimization: Pre-computed constants
double half_resolution;        // resolution / 2 (cached for fast boundary checks)  
double inv_resolution;         // 1 / resolution (cached for fast division)
double height_threshold_1;     // resolution * 1 (cached threshold for grid search)
double height_threshold_3;     // resolution * 3 (cached threshold for fallback search)

double expansionCoefficient = 1;
double expansion = 1;
double publish_rate = 2.0;  // Publishing rate in Hz

tf::TransformListener *listener_ptr;
sensor_msgs::PointCloud2 worldPoints;

// Double-buffering mechanism for worldCloud to prevent callback reentry issues
pcl::PointCloud<pcl::PointXYZ> worldCloud_buffer[2];  // Two buffers for double buffering
std::atomic<int> worldCloud_read_index(0);  // Index of buffer currently being read (0 or 1)
std::atomic<bool> worldCloud_initialized(false);  // Flag indicating if worldCloud has been initialized
std::mutex worldCloud_swap_mutex;  // Mutex for swapping buffers

// Atomic flag to prevent lidar callback reentry
std::atomic<bool> lidar_callback_running(false);

// Robot position caching for TF optimization
static tf::StampedTransform robot_transform_cache;
static ros::Time last_robot_transform_time;
static const double ROBOT_TRANSFORM_CACHE_DURATION = 0.1; // 100ms cache duration
static bool robot_transform_cache_valid = false;

// Local map optimization variables
pcl::PointCloud<pcl::PointXYZ> localWorldCloud;
GridIndex3D gridIndex(0.1); // Grid index with 0.1m resolution for fast lookup
Vector3d lastRobotPosition(std::numeric_limits<double>::max(), 
                          std::numeric_limits<double>::max(), 
                          std::numeric_limits<double>::max());
bool localMapInitialized = false;
bool gridIndexInitialized = false;
double localMapSize_x, localMapSize_y, localMapSize_z_l, localMapSize_z_u, cacheThreshold;
double z_compensation;

// Multi-layer parameters
double layer_gap_threshold = 0.3;      // Height gap to split into different layers (m)
size_t layer_merge_min_points = 3;     // Minimum points to keep a layer (filter noise)
double layer_height_tolerance = 0.15;  // Height tolerance for matching (m)

void updateLocalMap(const Vector3d& robotPosition);
void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points);

void updateLocalMap(const Vector3d& robotPosition) {
  DEBUG_LOG("updateLocalMap called with robot position: (%.2f, %.2f, %.2f)",
           robotPosition(0), robotPosition(1), robotPosition(2));

  // Get the current read index to access the correct buffer
  int read_idx = worldCloud_read_index.load();
  pcl::PointCloud<pcl::PointXYZ>& current_worldCloud = worldCloud_buffer[read_idx];

  DEBUG_LOG("World cloud size: %zu (using buffer %d)", current_worldCloud.size(), read_idx);

  if (!worldCloud_initialized.load() || current_worldCloud.empty()) {
    ROS_WARN("World cloud is empty or not initialized, cannot update local map");
    return;
  }

  // Check if we need to update the cache
  double distance = (robotPosition - lastRobotPosition).norm();
  if (localMapInitialized && distance < cacheThreshold) {
    return; // Use cached data
  }

  DEBUG_LOG("Updating local map at robot position: (%.2f, %.2f, %.2f)",
           robotPosition(0), robotPosition(1), robotPosition(2));

  // CRITICAL FIX: Create a local copy of worldCloud to prevent iterator invalidation
  // This prevents crashes if rcvWorldCallBack is called during iteration
  pcl::PointCloud<pcl::PointXYZ> worldCloudCopy;
  try {
    worldCloudCopy = current_worldCloud;  // Deep copy
    DEBUG_LOG("Created local copy of worldCloud with %zu points", worldCloudCopy.size());
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to copy worldCloud: %s", e.what());
    return;
  }

  // Clear previous local map
  localWorldCloud.clear();

  // Define local map bounds based on robot position
  double local_map_x_l = robotPosition(0) - localMapSize_x/2; // -5m
  double local_map_x_u = robotPosition(0) + localMapSize_x/2; // +5m
  double local_map_y_l = robotPosition(1) - localMapSize_y/2; // -5m
  double local_map_y_u = robotPosition(1) + localMapSize_y/2; // +5m
  double local_map_z_l = robotPosition(2) + localMapSize_z_l; // -0.5m below robot
  double local_map_z_u = robotPosition(2) + localMapSize_z_u; // +2.0m above robot

  // Extract local region from world cloud COPY (not the original!)
  for (const auto& pt : worldCloudCopy) {
    if (pt.x >= local_map_x_l && pt.x <= local_map_x_u &&
        pt.y >= local_map_y_l && pt.y <= local_map_y_u &&
        pt.z >= local_map_z_l && pt.z <= local_map_z_u) {
      localWorldCloud.push_back(pt);
    }
  }

  localWorldCloud.width = localWorldCloud.size();
  localWorldCloud.height = 1;
  localWorldCloud.is_dense = false;

  // Build grid index for fast spatial queries
  if (!localWorldCloud.empty()) {
    try {
      gridIndex.setGridSize(resolution); // Use same resolution as obstacle detection
      gridIndex.setLayerParameters(layer_gap_threshold, layer_merge_min_points, layer_height_tolerance);
      gridIndex.buildGrid(localWorldCloud);
      gridIndexInitialized = true;
      DEBUG_LOG("Grid index rebuilt with %zu points, grid size: %.3f",
               gridIndex.size(), resolution);
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to build grid index: %s", e.what());
      gridIndexInitialized = false;
    }
  } else {
    gridIndexInitialized = false;
  }

  localMapInitialized = true;
  lastRobotPosition = robotPosition;

  DEBUG_LOG("Local map updated: %zu points (from %zu world points)",
           localWorldCloud.size(), worldCloudCopy.size());
}

void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points)
{
  // CRITICAL FIX: Prevent callback reentry to avoid race conditions
  // If callback is already running, skip this call to prevent memory corruption
  if (lidar_callback_running.exchange(true)) {
    ROS_WARN_THROTTLE(1.0, "Lidar callback already running, skipping this iteration to prevent race conditions");
    return;
  }

  // Get the current read index for worldCloud buffer
  int read_idx = worldCloud_read_index.load();
  pcl::PointCloud<pcl::PointXYZ>& current_worldCloud = worldCloud_buffer[read_idx];

  if (!worldCloud_initialized.load() || current_worldCloud.empty()) {
    ROS_WARN("World cloud is empty or not initialized, skipping obstacle processing");
    lidar_callback_running = false;  // Release lock before returning
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(lidar_points, *cloud);
  
  if (cloud->empty()) {
    ROS_WARN("Received empty point cloud");
    lidar_callback_running = false;  // Release lock before returning
    return;
  }

  // Print point cloud statistics (debug only)
  if (!cloud->empty()) {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    DEBUG_LOG("Point cloud: %zu points, leaf_size: %f", cloud->size(), leaf_size);
    DEBUG_LOG("Point cloud range: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]", 
             min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
  }

  // Use custom spatial downsampling for reliable performance
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_VoxelGrid = spatial_downsample(cloud, leaf_size);
  
  DEBUG_LOG("Spatial downsampling completed: %zu points", cloud_after_VoxelGrid->size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_after_PassThrough->header = cloud_after_VoxelGrid->header;
  cloud_after_PassThrough->is_dense = false;
  
  // Manual PassThrough filtering to avoid PCL issues
  for (const auto& pt : cloud_after_VoxelGrid->points) {
    if (pt.x >= local_x_l && pt.x <= local_x_u &&
        pt.y >= local_y_l && pt.y <= local_y_u &&
        pt.z >= local_z_l && pt.z <= local_z_u) {
      cloud_after_PassThrough->points.push_back(pt);
    }
  }
  cloud_after_PassThrough->width = cloud_after_PassThrough->points.size();
  cloud_after_PassThrough->height = 1;

  DEBUG_LOG("After manual PassThrough filtering: %zu points", cloud_after_PassThrough->size());

  if (cloud_after_PassThrough->empty()) {
    ROS_WARN("Point cloud empty after PassThrough filtering");
    lidar_callback_running = false;  // Release lock before returning
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);

  Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
  Vector3d upperbound(local_x_u, local_y_u, local_z_u);

  // 优化：重用World对象，避免每次回调重新分配内存
  if (!local_world_initialized || local_world_reusable == NULL) {
    if (local_world_reusable != NULL) {
      delete local_world_reusable;
    }
    local_world_reusable = new World(resolution);
    local_world_reusable->initGridMap(lowerbound, upperbound);
    local_world_initialized = true;
    DEBUG_LOG("Created new reusable local world: bounds[%.2f,%.2f,%.2f] to [%.2f,%.2f,%.2f]", 
             lowerbound(0), lowerbound(1), lowerbound(2), 
             upperbound(0), upperbound(1), upperbound(2));
  } else {
    // 重用现有对象，只需重置网格状态为全True（空闲）
    Eigen::Vector3i idx_count = local_world_reusable->idx_count_;
    for (int i = 0; i < idx_count(0); i++) {
      for (int j = 0; j < idx_count(1); j++) {
        memset(local_world_reusable->grid_map_[i][j], true, idx_count(2) * sizeof(bool));
      }
    }
  }

  for (const auto &pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world_reusable->isFree(obstacle))
    {
      local_world_reusable->setObs(obstacle);
      Vector3d obstacle_round = local_world_reusable->coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }
  
  cloud_filt->width = cloud_filt->size();
  cloud_filt->height = 1;
  cloud_filt->is_dense = false;
  
  DEBUG_LOG("After local world processing: %zu points in cloud_filt", cloud_filt->size());

  try {
    listener_ptr->waitForTransform(map_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
  } catch (tf::TransformException &ex) {
    ROS_ERROR("TF transform failed: %s", ex.what());
    lidar_callback_running = false;  // Release lock before returning
    return;
  }

  // Get robot position for local map update with caching
  Vector3d robotPosition;
  ros::Time current_time = ros::Time::now();
  
  // Check if cached robot transform is still valid
  bool use_cached_transform = robot_transform_cache_valid && 
                             (current_time - last_robot_transform_time).toSec() < ROBOT_TRANSFORM_CACHE_DURATION;
  
  if (!use_cached_transform) {
    // Update robot transform cache
    try {
      listener_ptr->lookupTransform(map_frame_id, base_frame_id, ros::Time(0), robot_transform_cache);
      last_robot_transform_time = current_time;
      robot_transform_cache_valid = true;
      DEBUG_LOG("Updated robot transform cache");
    } catch (tf::TransformException &ex) {
      ROS_ERROR_THROTTLE(1.0, "Robot position transform failed: %s", ex.what());
      lidar_callback_running = false;  // Release lock before returning
      return;
    }
  }
  
  // Use cached transform to get robot position
  tf::Vector3 robot_origin = robot_transform_cache.getOrigin();
  robotPosition = Vector3d(robot_origin.x(), robot_origin.y(), robot_origin.z());
  
  // Update local map based on robot position
  updateLocalMap(robotPosition);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran_cost(new pcl::PointCloud<pcl::PointXYZ>);

  std_msgs::Float32MultiArray obs_array;
  
  DEBUG_LOG("Starting obstacle detection loop with %zu filtered points", cloud_filt->size());
  
  // Log which detection method will be used
  if (gridIndexInitialized && !gridIndex.empty()) {
    ROS_INFO_THROTTLE(5.0, "Using GRID INDEX detection (threshold: %.2fm) - %zu points in grid", 
                      height_threshold_1, gridIndex.size());
  } else {
    ROS_INFO_THROTTLE(5.0, "Using FALLBACK detection (threshold: %.2fm) - Reason: %s", 
                      height_threshold_3, 
                      !gridIndexInitialized ? "Grid index not initialized" : "Grid index empty");
  }
  
  // 优化：批量TF变换 - 获取一次变换矩阵并批量应用
  tf::StampedTransform transform;
  bool transform_available = false;
  try {
    listener_ptr->lookupTransform(map_frame_id, lidar_frame_id, ros::Time(0), transform);
    transform_available = true;
    DEBUG_LOG("Successfully obtained TF transform from %s to %s", lidar_frame_id.c_str(), map_frame_id.c_str());
  } catch (tf::TransformException &ex) {
    ROS_ERROR_THROTTLE(1.0, "Failed to get TF transform: %s", ex.what());
    lidar_callback_running = false;  // Release lock before returning
    return;
  }

  // 预先转换所有点云到map坐标系
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_transformed->reserve(cloud_filt->size());
  
  if (transform_available) {
    for (const auto &pt : cloud_filt->points) {
      tf::Vector3 point_in(pt.x, pt.y, pt.z);
      tf::Vector3 point_out = transform * point_in;
      
      pcl::PointXYZ pt_transformed;
      pt_transformed.x = point_out.x();
      pt_transformed.y = point_out.y();
      pt_transformed.z = point_out.z();
      cloud_transformed->push_back(pt_transformed);
    }
  }
  
  DEBUG_LOG("Batch transformation completed: %zu points transformed", cloud_transformed->size());
  
  int loop_count = 0;
  
  // 设置OpenMP线程数（可通过环境变量或参数配置）
  int num_threads = omp_get_max_threads();
  // int num_threads = 1;
  DEBUG_LOG("Using %d OpenMP threads for parallel obstacle detection", num_threads);
  
  // 预分配线程局部存储，避免频繁的锁竞争
  std::vector<std::vector<pcl::PointXYZ>> thread_cloud_tran(num_threads);
  std::vector<std::vector<pcl::PointXYZ>> thread_cloud_tran_cost(num_threads);
  std::vector<std::vector<float>> thread_obs_array(num_threads);
  
  #pragma omp parallel for schedule(dynamic, 100) num_threads(num_threads) \
          shared(cloud_transformed, cloud_filt, gridIndex, localWorldCloud, worldCloud_buffer, worldCloud_read_index)
  for (size_t i = 0; i < cloud_transformed->size(); ++i)
  {
    int thread_id = omp_get_thread_num();
    
    // 优化：直接使用已经批量变换后的点
    const pcl::PointXYZ& original_pt = cloud_filt->points[i];
    const pcl::PointXYZ& _pt = cloud_transformed->points[i];
    
    // Debug first few points (线程安全的调试输出) - disabled for performance
    #if DEBUG_OBSTACLE_PROCESSOR
    #pragma omp critical(debug_output)
    {
      static int total_debug_count = 0;
      if (total_debug_count < 3) {
        ROS_INFO("Thread[%d] Loop[%zu]: Original point (%.3f, %.3f, %.3f) -> Transformed (%.3f, %.3f, %.3f)", 
                 thread_id, i, original_pt.x, original_pt.y, original_pt.z, _pt.x, _pt.y, _pt.z);
        total_debug_count++;
      }
    }
    #endif

    // Use grid index for efficient spatial queries if available
    bool obstacleFound = false;
    
    // Debug output for first few points (线程安全) - disabled for performance
    #if DEBUG_OBSTACLE_PROCESSOR
    bool is_debug_point = false;
    int local_debug_id = -1;
    #pragma omp critical(debug_counter)
    {
      static int debug_count = 0;
      is_debug_point = (debug_count < 5);
      if (is_debug_point) {
        local_debug_id = debug_count;
        debug_count++;
      }
    }
    #else
    bool is_debug_point = false;
    int local_debug_id = -1;
    #endif
    
    if (gridIndexInitialized && !gridIndex.empty()) {
      try {
        // Multi-layer detection: Get all height layers for this XY grid cell
        std::vector<HeightLayer> layers = gridIndex.getLayersInGridCell(_pt);

        #if DEBUG_OBSTACLE_PROCESSOR
        if (is_debug_point) {
          ROS_INFO("Debug[%d]: Query point (%.3f, %.3f, %.3f), found %zu height layers",
                   local_debug_id, _pt.x, _pt.y, _pt.z, layers.size());
        }
        #endif

        if (layers.empty()) {
          // No layers found - treat as potential obstacle (will go to fallback)
          #if DEBUG_OBSTACLE_PROCESSOR
          if (is_debug_point) {
            ROS_INFO("Debug[%d]: No layers found, will check fallback", local_debug_id);
          }
          #endif
        } else {
          // Find the closest layer by centroid height
          double query_z = _pt.z + z_compensation;
          double min_distance = std::numeric_limits<double>::max();
          const HeightLayer* closest_layer = nullptr;

          for (const auto& layer : layers) {
            double distance = std::abs(query_z - layer.centroid_z);
            if (distance < min_distance) {
              min_distance = distance;
              closest_layer = &layer;
            }
          }

          #if DEBUG_OBSTACLE_PROCESSOR
          if (is_debug_point && closest_layer) {
            ROS_INFO("Debug[%d]: Closest layer: centroid_z=%.3f, min_z=%.3f, max_z=%.3f, count=%zu",
                     local_debug_id, closest_layer->centroid_z, closest_layer->min_z,
                     closest_layer->max_z, closest_layer->count);
          }
          #endif

          // Check if query point height falls within the extended range of closest layer
          if (closest_layer) {
            double layer_min_extended = closest_layer->min_z - layer_height_tolerance;
            double layer_max_extended = closest_layer->max_z + layer_height_tolerance;

            if (query_z >= layer_min_extended && query_z <= layer_max_extended) {
              // Point matches an existing layer - NOT an obstacle
              #if DEBUG_OBSTACLE_PROCESSOR
              if (is_debug_point) {
                ROS_INFO("Debug[%d]: Point matches layer [%.3f, %.3f], query_z=%.3f - NOT obstacle",
                         local_debug_id, layer_min_extended, layer_max_extended, query_z);
              }
              #endif
              obstacleFound = false;
            } else {
              // Point is outside all layer ranges - check if height difference exceeds threshold
              double height_diff_below = query_z - closest_layer->max_z;
              double height_diff_above = closest_layer->min_z - query_z;
              double min_height_diff = std::min(std::abs(height_diff_below), std::abs(height_diff_above));

              #if DEBUG_OBSTACLE_PROCESSOR
              if (is_debug_point) {
                ROS_INFO("Debug[%d]: Point outside layer range. height_diff_below=%.3f, height_diff_above=%.3f, min_diff=%.3f, threshold=%.3f",
                         local_debug_id, height_diff_below, height_diff_above, min_height_diff, height_threshold_1);
              }
              #endif

              if (height_diff_below >= height_threshold_1 || height_diff_above >= height_threshold_1) {
                // Height difference is significant - mark as obstacle
                #if DEBUG_OBSTACLE_PROCESSOR
                if (is_debug_point) {
                  ROS_INFO("Debug[%d]: OBSTACLE DETECTED! Height diff exceeds threshold", local_debug_id);
                }
                #endif
                obstacleFound = true;
              } else {
                // Height difference is small - not an obstacle
                obstacleFound = false;
              }
            }
          }
        }
      } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Grid index search failed: %s", e.what());
        gridIndexInitialized = false;
      }
    } else {
      // Fallback to direct search on local map
      DEBUG_LOG_THROTTLE(5.0, "Grid index not available, using fallback search");
      // CRITICAL FIX: Get current worldCloud buffer instead of using static reference
      int current_read_idx = worldCloud_read_index.load();
      pcl::PointCloud<pcl::PointXYZ>& mapToUse = localMapInitialized ? localWorldCloud : worldCloud_buffer[current_read_idx];
      
      #if DEBUG_OBSTACLE_PROCESSOR
      if (is_debug_point) {
        ROS_INFO("Debug[%d]: Using fallback search on %zu points", local_debug_id, mapToUse.size());
      }
      #endif
      
      for (const auto &pt : mapToUse) {
        // Fast grid boundary check for fallback search  
        double dx = _pt.x - pt.x;
        double dy = _pt.y - pt.y;
        double half_res = half_resolution; // Use pre-computed value
        if (std::abs(dx) <= half_res && std::abs(dy) <= half_res) {
          
          double heightDiff = (_pt.z + z_compensation) - pt.z;
          double threshold = height_threshold_3; // Pre-computed fallback threshold
          
          #if DEBUG_OBSTACLE_PROCESSOR
          if (is_debug_point) {
            // Use already computed dx, dy for debug (no sqrt needed)
            double xy_dist_sq = dx*dx + dy*dy;
            ROS_INFO("Debug[%d]: Fallback - Map point (%.3f, %.3f, %.3f), XY dist²: %.3f, height diff: %.3f, threshold: %.3f", 
                     local_debug_id, pt.x, pt.y, pt.z, xy_dist_sq, heightDiff, threshold);
          }
          #endif
          
          if (heightDiff >= threshold) {
            #if DEBUG_OBSTACLE_PROCESSOR
            if (is_debug_point) {
              ROS_INFO("Debug[%d]: FALLBACK OBSTACLE DETECTED! Height diff: %.3f", local_debug_id, heightDiff);
            }
            #endif
            obstacleFound = true;
            break;
          }
        }
      }
    }
    
    #if DEBUG_OBSTACLE_PROCESSOR
    if (is_debug_point) {
      ROS_INFO("Debug[%d]: Final result - obstacleFound: %s", local_debug_id, obstacleFound ? "TRUE" : "FALSE");
    }
    #endif
    
    // Early exit debug for first few loops (thread-safe debug) - disabled for performance
    #if DEBUG_OBSTACLE_PROCESSOR
    #pragma omp critical(early_debug)
    {
      static int early_debug_count = 0;
      if (early_debug_count < 3 && !obstacleFound) {
        ROS_INFO("Thread[%d] Loop[%zu]: No obstacle found for this point", thread_id, i);
        early_debug_count++;
      }
    }
    #endif
    
    if (obstacleFound) {
      // 使用线程局部存储，避免锁竞争
      thread_cloud_tran_cost[thread_id].push_back(_pt);
      
      // 使用优化的障碍物扩展函数
      ObstacleExpander::expandObstacle(_pt, thread_cloud_tran[thread_id], thread_obs_array[thread_id]);
    }

  }
  
  // 合并线程局部结果到最终输出（在并行区域之外）
  DEBUG_LOG("Merging results from %d threads...", num_threads);
  
  // 估算总大小以优化内存分配
  size_t total_cloud_tran_size = 0, total_cloud_tran_cost_size = 0, total_obs_array_size = 0;
  for (int t = 0; t < num_threads; ++t) {
    total_cloud_tran_size += thread_cloud_tran[t].size();
    total_cloud_tran_cost_size += thread_cloud_tran_cost[t].size();
    total_obs_array_size += thread_obs_array[t].size();
  }
  
  // 预分配内存
  cloud_tran->points.reserve(total_cloud_tran_size);
  cloud_tran_cost->points.reserve(total_cloud_tran_cost_size);
  obs_array.data.reserve(total_obs_array_size);
  
  // 合并点云数据
  for (int t = 0; t < num_threads; ++t) {
    cloud_tran->points.insert(cloud_tran->points.end(), 
                              thread_cloud_tran[t].begin(), 
                              thread_cloud_tran[t].end());
    cloud_tran_cost->points.insert(cloud_tran_cost->points.end(), 
                                   thread_cloud_tran_cost[t].begin(), 
                                   thread_cloud_tran_cost[t].end());
    obs_array.data.insert(obs_array.data.end(), 
                          thread_obs_array[t].begin(), 
                          thread_obs_array[t].end());
  }
  
  // 更新点云属性
  cloud_tran->width = cloud_tran->size();
  cloud_tran->height = 1;
  cloud_tran->is_dense = false;
  
  cloud_tran_cost->width = cloud_tran_cost->size();
  cloud_tran_cost->height = 1;
  cloud_tran_cost->is_dense = false;

  // Debug output for final results
  DEBUG_LOG("Final obstacle detection results:");
  DEBUG_LOG("  - Processed %zu points with parallel processing, batch TF transformation used", cloud_transformed->size());
  DEBUG_LOG("  - cloud_tran: %zu points", cloud_tran->size());
  DEBUG_LOG("  - cloud_tran_cost: %zu points", cloud_tran_cost->size());
  DEBUG_LOG("  - obs_array: %zu elements (%zu points)", obs_array.data.size(), obs_array.data.size()/3);
  DEBUG_LOG("  - gridIndexInitialized: %s", gridIndexInitialized ? "true" : "false");
  DEBUG_LOG("  - localMapInitialized: %s", localMapInitialized ? "true" : "false");
  if (localMapInitialized) {
    DEBUG_LOG("  - localWorldCloud size: %zu", localWorldCloud.size());
  }
  if (gridIndexInitialized) {
    DEBUG_LOG("  - gridIndex size: %zu", gridIndex.size());
  }
  
  sensor_msgs::PointCloud2 obs_vis;
  pcl::toROSMsg(*cloud_tran, obs_vis);
  obs_vis.header.frame_id = map_frame_id;
  obs_pub.publish(obs_vis);

  obs_array_pub.publish(obs_array);

  sensor_msgs::PointCloud2 obs_cost;
  pcl::toROSMsg(*cloud_tran_cost, obs_cost);
  obs_cost.header.frame_id = map_frame_id;
  obs_cost_pub.publish(obs_cost);

  // Release lock at the end of callback
  lidar_callback_running = false;
  DEBUG_LOG("Lidar callback completed successfully");
}

void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  // CRITICAL FIX: Use double-buffering to prevent iterator invalidation
  // Write to the inactive buffer, then atomically swap the read index

  ROS_INFO_THROTTLE(5.0, "Received new world map with %zu points",
                    static_cast<std::size_t>(pointcloud_map.width) * pointcloud_map.height);

  // Lock to prevent simultaneous buffer swaps
  std::lock_guard<std::mutex> lock(worldCloud_swap_mutex);

  // Get current read index and calculate write index
  int current_read_idx = worldCloud_read_index.load();
  int write_idx = 1 - current_read_idx;  // Toggle between 0 and 1

  DEBUG_LOG("WorldCloud callback: current read_idx=%d, writing to buffer %d",
           current_read_idx, write_idx);

  // Write to the INACTIVE buffer (safe from concurrent reads)
  try {
    pcl::fromROSMsg(pointcloud_map, worldCloud_buffer[write_idx]);

    DEBUG_LOG("Successfully wrote %zu points to worldCloud buffer %d",
             worldCloud_buffer[write_idx].size(), write_idx);

    // Atomically swap the read index to point to the newly written buffer
    // From this point, all readers will use the new buffer
    worldCloud_read_index.store(write_idx);
    worldCloud_initialized.store(true);

    DEBUG_LOG("Swapped worldCloud read index to %d. Old buffer %d is now write buffer.",
             write_idx, current_read_idx);

  } catch (const std::exception& e) {
    ROS_ERROR("Failed to convert world cloud: %s", e.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_processor");
  ros::NodeHandle nh("~");

  // pt_sub = nh.subscribe("/cloud_registered_body", 1, rcvLidarCallBack);
  // pt_sub = nh.subscribe("/cloud_registered_body_1", 1, rcvLidarCallBack);
  // pt_sub = nh.subscribe("/cloud_registered_body", 1, rcvLidarCallBack);
  pt_sub = nh.subscribe("/points_e1r_front", 1, rcvLidarCallBack); // for simulation
  // world_sub = nh.subscribe("/3dmap", 1, rcvWorldCallBack);
  world_sub = nh.subscribe("/map", 1, rcvWorldCallBack);
  // world_sub = nh.subscribe("/planning_3d_PRM_node/pcd_map", 1, rcvWorldCallBack); // for simulation

  obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);
  obs_cost_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_cost", 1);
  obs_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/obs_raw", 1);

  nh.getParam("map/map_frame_id", map_frame_id);
  nh.getParam("map/lidar_frame_id", lidar_frame_id);
  nh.getParam("map/base_frame_id", base_frame_id);
  nh.param("map/resolution", resolution, 0.1);

  nh.param("map/expansionCoefficient", expansionCoefficient, 1.0);
  nh.param("map/leaf_size", leaf_size, 0.2);
  nh.param("map/local_x_l", local_x_l, -2.0);
  nh.param("map/local_x_u", local_x_u, 2.0);
  nh.param("map/local_y_l", local_y_l, -2.0);
  nh.param("map/local_y_u", local_y_u, 2.0);
  nh.param("map/local_z_l", local_z_l, -0.3);
  nh.param("map/local_z_u", local_z_u, 0.5);

  // Local map parameters
  nh.param("map/localMapSize_x", localMapSize_x, 10.0);
  nh.param("map/localMapSize_y", localMapSize_y, 10.0);
  nh.param("map/localMapSize_z_l", localMapSize_z_l, 0.5);
  nh.param("map/localMapSize_z_u", localMapSize_z_u, 2.0);
  nh.param("map/cacheThreshold", cacheThreshold, 1.0);
  
  // Z-axis compensation parameter
  nh.param("map/z_compensation", z_compensation, 0.0);
  
  // Height threshold parameters (independent values in meters)
  nh.param("map/height_threshold_grid", height_threshold_1, 0.3);
  nh.param("map/height_threshold_fallback", height_threshold_3, 0.9);

  // Multi-layer obstacle detection parameters
  nh.param("map/layer_gap_threshold", layer_gap_threshold, 0.3);
  nh.param("map/layer_merge_min_points", (int&)layer_merge_min_points, 3);
  nh.param("map/layer_height_tolerance", layer_height_tolerance, 0.15);

  // Publishing rate parameter
  nh.param("map/publish_rate", publish_rate, 2.0);

  expansion = resolution * expansionCoefficient;
  
  // Initialize pre-computed constants for performance optimization
  half_resolution = resolution * 0.5;
  inv_resolution = 1.0 / resolution;

  // Initialize optimized obstacle expander
  ObstacleExpander::initialize(expansion);

  // Log downsampling configuration
  ROS_INFO("Downsampling configuration:");
  ROS_INFO("  Using custom voxel grid (memory-safe, no PCL dependencies)");
  ROS_INFO("  Leaf size: %.3f", leaf_size);

  // Log multi-layer detection configuration
  ROS_INFO("Multi-layer obstacle detection configuration:");
  ROS_INFO("  layer_gap_threshold: %.3f m (height gap to split layers)", layer_gap_threshold);
  ROS_INFO("  layer_merge_min_points: %zu (minimum points to keep a layer)", layer_merge_min_points);
  ROS_INFO("  layer_height_tolerance: %.3f m (height tolerance for matching)", layer_height_tolerance);
  ROS_INFO("  height_threshold_grid: %.3f m (grid detection threshold)", height_threshold_1);
  ROS_INFO("  height_threshold_fallback: %.3f m (fallback detection threshold)", height_threshold_3);

  tf::TransformListener listener;
  listener_ptr = &listener;
  world = new World(resolution);

  // while (ros::ok())
  // {
  //   timeval start;
  //   gettimeofday(&start, NULL);
  //   ros::spinOnce();
  //   double ms;
  //   do
  //   {
  //     timeval end;
  //     gettimeofday(&end, NULL);
  //     ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
  //   } while (ms < 50);
  // }
  ros::Rate rate(publish_rate); // Configurable rate
  ROS_INFO("Obstacle processor running at %.1f Hz", publish_rate);
  while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
  }
  
  // 清理资源
  if (local_world_reusable != NULL) {
    delete local_world_reusable;
    local_world_reusable = NULL;
  }
  if (world != NULL) {
    delete world;
    world = NULL;
  }
  
  return 0;
}
