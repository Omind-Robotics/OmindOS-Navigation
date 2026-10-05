#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class M20LidarFusion
{
public:
  M20LidarFusion()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("front_topic", front_topic_, "/m20/lidar/front");
    private_nh_.param<std::string>("rear_topic", rear_topic_, "/m20/lidar/rear");
    private_nh_.param<std::string>("output_topic", output_topic_, "/m20/lidar/fused");
    private_nh_.param<std::string>("output_frame", output_frame_, "base_link");
    private_nh_.param<double>("max_sync_diff", max_sync_diff_, 0.005);
    private_nh_.param<int>("queue_size", queue_size_, 10);

    queue_size_ = std::max(2, queue_size_);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    front_subscriber_ = nh_.subscribe(front_topic_, queue_size_, &M20LidarFusion::frontCallback, this);
    rear_subscriber_ = nh_.subscribe(rear_topic_, queue_size_, &M20LidarFusion::rearCallback, this);

    ROS_INFO_STREAM("M20 LiDAR fusion: " << front_topic_ << " + " << rear_topic_
                    << " -> " << output_topic_ << ", max_sync_diff="
                    << max_sync_diff_ * 1000.0 << " ms");
  }

private:
  using CloudConstPtr = sensor_msgs::PointCloud2ConstPtr;

  void frontCallback(const CloudConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pushBounded(front_queue_, message);
    matchAndPublish();
  }

  void rearCallback(const CloudConstPtr &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pushBounded(rear_queue_, message);
    matchAndPublish();
  }

  void pushBounded(std::deque<CloudConstPtr> &queue, const CloudConstPtr &message)
  {
    queue.push_back(message);
    while (static_cast<int>(queue.size()) > queue_size_)
    {
      queue.pop_front();
    }
  }

  static bool fieldsMatch(const sensor_msgs::PointCloud2 &front,
                          const sensor_msgs::PointCloud2 &rear)
  {
    if (front.point_step != rear.point_step || front.is_bigendian != rear.is_bigendian ||
        front.fields.size() != rear.fields.size())
    {
      return false;
    }

    for (std::size_t index = 0; index < front.fields.size(); ++index)
    {
      const auto &lhs = front.fields[index];
      const auto &rhs = rear.fields[index];
      if (lhs.name != rhs.name || lhs.offset != rhs.offset || lhs.datatype != rhs.datatype ||
          lhs.count != rhs.count)
      {
        return false;
      }
    }
    return true;
  }

  void matchAndPublish()
  {
    while (!front_queue_.empty() && !rear_queue_.empty())
    {
      const double difference =
          (front_queue_.front()->header.stamp - rear_queue_.front()->header.stamp).toSec();
      if (std::abs(difference) <= max_sync_diff_)
      {
        const auto front = front_queue_.front();
        const auto rear = rear_queue_.front();
        front_queue_.pop_front();
        rear_queue_.pop_front();
        publishFused(front, rear, difference);
      }
      else if (difference < 0.0)
      {
        front_queue_.pop_front();
        ROS_WARN_THROTTLE(2.0, "Dropping unmatched M20 front LiDAR frame");
      }
      else
      {
        rear_queue_.pop_front();
        ROS_WARN_THROTTLE(2.0, "Dropping unmatched M20 rear LiDAR frame");
      }
    }
  }

  void publishFused(const CloudConstPtr &front, const CloudConstPtr &rear, double sync_difference)
  {
    if (!fieldsMatch(*front, *rear))
    {
      ROS_ERROR_THROTTLE(2.0, "M20 front/rear PointCloud2 field layouts do not match");
      return;
    }

    const std::uint64_t front_points =
        static_cast<std::uint64_t>(front->width) * static_cast<std::uint64_t>(front->height);
    const std::uint64_t rear_points =
        static_cast<std::uint64_t>(rear->width) * static_cast<std::uint64_t>(rear->height);
    const std::uint64_t front_bytes = front_points * front->point_step;
    const std::uint64_t rear_bytes = rear_points * rear->point_step;
    if (front->data.size() != front_bytes || rear->data.size() != rear_bytes)
    {
      ROS_ERROR_THROTTLE(2.0, "M20 PointCloud2 contains unsupported row padding");
      return;
    }

    sensor_msgs::PointCloud2 fused = *front;
    fused.header.stamp = std::min(front->header.stamp, rear->header.stamp);
    fused.header.frame_id = output_frame_;
    fused.height = 1;
    fused.width = static_cast<std::uint32_t>(front_points + rear_points);
    fused.row_step = fused.point_step * fused.width;
    fused.is_dense = front->is_dense && rear->is_dense;
    fused.data.clear();
    fused.data.reserve(front->data.size() + rear->data.size());
    fused.data.insert(fused.data.end(), front->data.begin(), front->data.end());
    fused.data.insert(fused.data.end(), rear->data.begin(), rear->data.end());
    publisher_.publish(fused);

    ROS_INFO_STREAM_THROTTLE(5.0, "M20 fused cloud: " << fused.width
                             << " points, sync difference="
                             << std::abs(sync_difference) * 1000.0 << " ms");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber front_subscriber_;
  ros::Subscriber rear_subscriber_;
  ros::Publisher publisher_;
  std::deque<CloudConstPtr> front_queue_;
  std::deque<CloudConstPtr> rear_queue_;
  std::mutex mutex_;
  std::string front_topic_;
  std::string rear_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double max_sync_diff_;
  int queue_size_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m20_lidar_fusion");
  M20LidarFusion fusion;
  ros::spin();
  return 0;
}
