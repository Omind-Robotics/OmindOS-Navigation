#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>

static const double EPS = 1e-6;

/*---------------- 工具：距离、贝塞尔采样 ----------------*/
inline double dist3D(const geometry_msgs::Point& a,
                     const geometry_msgs::Point& b)
{
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/* 通用 n 阶贝塞尔采样器，P 大小 = 阶数+1 */
geometry_msgs::Point bezierN(const std::vector<geometry_msgs::Point>& P,
                             double t)
{
  /* 用 De Casteljau 算法，数值稳定 */
  std::vector<geometry_msgs::Point> Q = P;
  int n = Q.size() - 1;
  for (int k = 1; k <= n; ++k)
    for (int i = 0; i <= n - k; ++i)
    {
      Q[i].x = (1 - t) * Q[i].x + t * Q[i + 1].x;
      Q[i].y = (1 - t) * Q[i].y + t * Q[i + 1].y;
      Q[i].z = (1 - t) * Q[i].z + t * Q[i + 1].z;
    }
  return Q[0];
}

/*---------------- 节点类 ----------------*/
class BezierOptimizer
{
public:
  BezierOptimizer()
  {
    ros::NodeHandle nh("~");
    nh.param("sample_rate", sample_rate_, 20);
    nh.param("q1_scale", q1_scale_, 0.3);
    nh.param("q1_max_ratio", q1_max_ratio_, 1.0);
    nh.param("enable_orientation", enable_orientation_, true);
    nh.param("end_strategy", end_strategy_, std::string("same_as_prev"));

    path_sub_ = nh.subscribe("/planned_path", 1,
                             &BezierOptimizer::pathCb, this);
    path_pub_ = nh.advertise<nav_msgs::Path>("/path_smooth", 1, true);
    ROS_INFO("[BezierOptimizer] ready.");
  }

private:
  int sample_rate_;
  double q1_scale_, q1_max_ratio_;
  bool enable_orientation_;
  std::string end_strategy_;
  ros::Subscriber path_sub_;
  ros::Publisher  path_pub_;

  /* 计算 Q1 辅助点，与原始代码一致 */
  geometry_msgs::Point computeQ1(const geometry_msgs::Point& P4,
                                 const geometry_msgs::Point& P5,
                                 const geometry_msgs::Point& Q2) const
  {
    geometry_msgs::Point dir;
    dir.x = P5.x - P4.x;
    dir.y = P5.y - P4.y;
    dir.z = P5.z - P4.z;
    double len = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
    if (len < EPS) len = EPS;
    double d = q1_scale_ * len;
    d = std::min(d, q1_max_ratio_ * len);

    geometry_msgs::Point Q1;
    Q1.x = P5.x + d * dir.x / len;
    Q1.y = P5.y + d * dir.y / len;
    Q1.z = P5.z + d * dir.z / len;

    if (dist3D(Q1, P5) >= dist3D(Q1, Q2))
    {
      double extra = dist3D(Q1, P5) - dist3D(Q1, Q2) + 0.01;
      Q1.x += extra * dir.x / len;
      Q1.y += extra * dir.y / len;
      Q1.z += extra * dir.z / len;
    }
    return Q1;
  }

  /* 路径回调 */
  void pathCb(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.empty()) return;

    /* 1. 提取原始坐标 */
    std::vector<geometry_msgs::Point> raw;
    raw.reserve(msg->poses.size());
    for (const auto& p : msg->poses) raw.push_back(p.pose.position);

    /* 2. 分段拟合 */
    std::vector<geometry_msgs::Point> tmp_pts;
    size_t seg_beg = 0, N = raw.size();
    while (seg_beg + 5 < N)               // 足够 6 个点 → 5 阶
    {
      std::vector<geometry_msgs::Point> P(raw.begin() + seg_beg,
                                          raw.begin() + seg_beg + 6);
      for (int k = 0; k <= sample_rate_; ++k)
        tmp_pts.push_back(bezierN(P, double(k) / sample_rate_));

      /* 插入 Q1 辅助点 */
      geometry_msgs::Point Q1 = computeQ1(P[4], P[5],
                                          (seg_beg + 6 < N) ? raw[seg_beg + 6]
                                                             : raw.back());
      raw.insert(raw.begin() + seg_beg + 6, Q1);
      N = raw.size();
      seg_beg += 6;
    }

    /* 3. 处理最后一段不足 6 个点 */
    if (seg_beg < N)
    {
      size_t cnt = N - seg_beg;   // 实际点数
      std::vector<geometry_msgs::Point> P(raw.begin() + seg_beg, raw.end());
      /* 若只有 1 个点，退化成“一阶”即直线插值到自己，采样只重复该点 */
      if (cnt == 1)
      {
        for (int k = 0; k <= sample_rate_; ++k)
          tmp_pts.push_back(P[0]);
      }
      else
      {
        /* cnt >=2，直接用 cnt-1 阶贝塞尔 */
        for (int k = 0; k <= sample_rate_; ++k)
          tmp_pts.push_back(bezierN(P, double(k) / sample_rate_));
      }
    }

    /* 4. 组装 Path 并计算方向 */
    nav_msgs::Path out;
    out.header = msg->header;
    out.poses.reserve(tmp_pts.size());

    for (size_t i = 0; i < tmp_pts.size(); ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position = tmp_pts[i];

      tf2::Vector3 dir;
      if (i + 1 < tmp_pts.size())
        dir = tf2::Vector3(tmp_pts[i+1].x - tmp_pts[i].x,
                           tmp_pts[i+1].y - tmp_pts[i].y,
                           tmp_pts[i+1].z - tmp_pts[i].z);
      else
      {
        if (end_strategy_ == "keep_original" &&
            i < msg->poses.size() &&
            std::abs(msg->poses[i].pose.orientation.w) > EPS)
        {
          ps.pose.orientation = msg->poses[i].pose.orientation;
          out.poses.push_back(ps);
          continue;
        }
        else
          dir = tf2::Vector3(tmp_pts[i].x - tmp_pts[i-1].x,
                             tmp_pts[i].y - tmp_pts[i-1].y,
                             tmp_pts[i].z - tmp_pts[i-1].z);
      }

      if (dir.length() < EPS) dir = tf2::Vector3(1, 0, 0);
      else                    dir.normalize();

      tf2::Vector3 z(0, 0, 1), y = z.cross(dir);
      if (y.length() < EPS)
      {
        y = tf2::Vector3(0, 1, 0);
        if (dir.z() > 0.9) y = tf2::Vector3(0, -1, 0);
      }
      y.normalize();
      tf2::Vector3 x = y.cross(z);
      tf2::Matrix3x3 R(x.x(), y.x(), z.x(),
                       x.y(), y.y(), z.y(),
                       x.z(), y.z(), z.z());
      tf2::Quaternion q;
      R.getRotation(q);
      q.normalize();

      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      out.poses.push_back(ps);
    }

    /* 5. 发布 */
    path_pub_.publish(out);
    ROS_INFO("[BezierOptimizer] published smooth path with %zu poses",
             out.poses.size());
  }
};

/*---------------- main ----------------*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bezier_path_optimizer");
  BezierOptimizer bezier_optimizer;
  ros::spin();
  return 0;
}