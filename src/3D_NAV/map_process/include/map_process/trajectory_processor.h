#ifndef TRAJECTORY_PROCESSOR_H
#define TRAJECTORY_PROCESSOR_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map_process {

/**
 * @brief Structure to represent robot pose
 */
struct RobotPose {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    RobotPose() : timestamp(0.0) {}
    RobotPose(double t, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
        : timestamp(t), position(pos), orientation(quat) {}
};

/**
 * @brief Class for processing robot trajectory and extracting seed points
 */
class TrajectoryProcessor {
public:
    /**
     * @brief Constructor
     */
    TrajectoryProcessor();
    
    /**
     * @brief Destructor
     */
    ~TrajectoryProcessor();
    
    /**
     * @brief Set trajectory file format
     * @param format Format type: 1 (timestamp x y z qx qy qz qw), 2 (x y z param1 param2 param3), 3 (cols 5-7 are xyz)
     */
    void setTrajectoryFormat(int format);
    
    /**
     * @brief Load trajectory from text file
     * @param filename Path to trajectory file
     * @return True if successful, false otherwise
     */
    bool loadTrajectory(const std::string& filename);
    
    /**
     * @brief Sample trajectory points at regular intervals
     * @param interval Distance interval for sampling (meters)
     * @return Vector of sampled poses
     */
    std::vector<RobotPose> sampleTrajectory(double interval = 1.0);
    
    /**
     * @brief Find seed points near trajectory positions
     * @param cloud Input point cloud
     * @param sampled_poses Sampled trajectory poses
     * @param search_radius Search radius around each pose
     * @param height_offset Height offset below robot position to search for ground points
     * @return Point cloud containing seed points
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr findSeedPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::vector<RobotPose>& sampled_poses,
        double search_radius = 0.5,
        double height_offset = 1.0
    );
    
    /**
     * @brief Get the loaded trajectory
     * @return Reference to trajectory vector
     */
    const std::vector<RobotPose>& getTrajectory() const { return trajectory_; }
    
    /**
     * @brief Get trajectory length
     * @return Total trajectory length in meters
     */
    double getTrajectoryLength() const;
    
private:
    std::vector<RobotPose> trajectory_;
    int trajectory_format_;  // Format type: 1, 2, or 3
    
    /**
     * @brief Calculate distance between two poses
     * @param pose1 First pose
     * @param pose2 Second pose
     * @return Distance in meters
     */
    double calculateDistance(const RobotPose& pose1, const RobotPose& pose2) const;
    
    /**
     * @brief Parse a line from trajectory file
     * @param line Text line to parse
     * @param pose Output pose
     * @return True if parsing successful
     */
    bool parsePoseLine(const std::string& line, RobotPose& pose) const;
};

} // namespace map_process

#endif // TRAJECTORY_PROCESSOR_H
