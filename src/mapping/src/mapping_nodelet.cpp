#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/int16_multi_array.hpp>

// 假设已迁移 quadrotor_msgs
#include <quadrotor_msgs/msg/occ_map3d.hpp> 

#include "mapping/mapping.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <atomic>
#include <thread>
#include <memory>

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
    ImageOdomSyncPolicy;
typedef message_filters::Synchronizer<ImageOdomSyncPolicy>
    ImageOdomSynchronizer;

struct CamConfig {
  double rate;
  double range;
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scaling_factor;
};

class MappingNode : public rclcpp::Node {
 private:
  CamConfig camConfig_;
  int down_sample_factor_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;

  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;

  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<ImageOdomSynchronizer> depth_odom_sync_Ptr_;
  
  rclcpp::Publisher<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  bool map_recieved_ = false;
  bool use_global_map_ = false;

  bool use_mask_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_;

  OccGridMap gridmap_;
  int inflate_size_;

  void depth_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                           const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    if (callback_lock_.test_and_set()) {
      return;
    }
    
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
    
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        callback_lock_.clear();
        return;
    }

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    int nr = depth_img.rows;
    int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;

    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z)) continue;
        if (z < depth_filter_mindist_) continue;

        double y = (i - camConfig_.cy) * z / camConfig_.fy;
        double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z);
        p = cam_q * p + cam_p;
        bool good_point = true;
        
        if (get_first_frame_) {
          Eigen::Vector3d p_rev_proj =
              last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) {
              good_point = false;
            }
          }
        }
        if (good_point) {
          obs_pts.push_back(p);
        }
      }
    }
    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;
    
    gridmap_.updateMap(cam_p, obs_pts);

    if (use_mask_) {
      while (target_lock_.test_and_set()) ;
      Eigen::Vector3d ld = target_odom_;
      Eigen::Vector3d ru = target_odom_;
      ld.x() -= 0.5; ld.y() -= 0.5; ld.z() -= 1.0;
      ru.x() += 0.5; ru.y() += 0.5; ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
      target_lock_.clear();
    }

    quadrotor_msgs::msg::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = this->now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_->publish(gridmap_msg);

    callback_lock_.clear();
  }

  void target_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msgPtr) {
    while (target_lock_.test_and_set()) ;
    target_odom_.x() = msgPtr->pose.pose.position.x;
    target_odom_.y() = msgPtr->pose.pose.position.y;
    target_odom_.z() = msgPtr->pose.pose.position.z;
    target_lock_.clear();
  }

  void map_call_back(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msgPtr) {
    if (map_recieved_) return;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      gridmap_.setOcc(p);
    }
    gridmap_.inflate(inflate_size_);
    RCLCPP_WARN(this->get_logger(), "[mapping] GLOBAL MAP RECEIVED!");
    map_recieved_ = true;
  }

  void global_map_timer_callback() {
    if (!map_recieved_) return;
    quadrotor_msgs::msg::OccMap3d gridmap_msg;
    gridmap_.to_msg(gridmap_msg);
    gridmap_msg.header.stamp = this->now();
    gridmap_msg.header.frame_id = "world";
    gridmap_inflate_pub_->publish(gridmap_msg);
  }

 public:
  MappingNode(const rclcpp::NodeOptions & options) : Node("mapping_node", options) {
    // Parameter declaration
    this->declare_parameter("cam2body_R", std::vector<double>());
    this->declare_parameter("cam2body_p", std::vector<double>());
    this->declare_parameter("use_global_map", false);
    this->declare_parameter("use_mask", false);
    this->declare_parameter("inflate_size", 2); 
    
    // Global map params
    this->declare_parameter("x_length", 10.0);
    this->declare_parameter("y_length", 10.0);
    this->declare_parameter("z_length", 3.0);
    
    // Local map / Camera params
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("camera_rate", 30.0);
    this->declare_parameter("camera_range", 4.0);
    this->declare_parameter("cam_width", 640);
    this->declare_parameter("cam_height", 480);
    this->declare_parameter("cam_fx", 320.0);
    this->declare_parameter("cam_fy", 320.0);
    this->declare_parameter("cam_cx", 320.0);
    this->declare_parameter("cam_cy", 240.0);
    this->declare_parameter("depth_scaling_factor", 1000.0);
    this->declare_parameter("down_sample_factor", 4);
    this->declare_parameter("local_x", 10.0);
    this->declare_parameter("local_y", 10.0);
    this->declare_parameter("local_z", 3.0);
    this->declare_parameter("depth_filter_tolerance", 0.1);
    this->declare_parameter("depth_filter_mindist", 0.2);
    this->declare_parameter("depth_filter_margin", 0);
    
    this->declare_parameter("p_min", -100);
    this->declare_parameter("p_max", 100);
    this->declare_parameter("p_hit", 10);
    this->declare_parameter("p_mis", -10);
    this->declare_parameter("p_occ", 50);
    this->declare_parameter("p_def", -50);

    // Initialization Logic
    std::vector<double> tmp;
    if (this->get_parameter("cam2body_R", tmp) && tmp.size() == 9) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
    } else {
        cam2body_R_.setIdentity();
    }
    if (this->get_parameter("cam2body_p", tmp) && tmp.size() == 3) {
      cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
    } else {
        cam2body_p_.setZero();
    }

    double res;
    this->get_parameter("use_global_map", use_global_map_);
    this->get_parameter("inflate_size", inflate_size_);
    this->get_parameter("resolution", res);

    if (use_global_map_) {
      double x, y, z;
      this->get_parameter("x_length", x);
      this->get_parameter("y_length", y);
      this->get_parameter("z_length", z);
      gridmap_.setup(res, Eigen::Vector3d(x, y, z), 10, true);
    } else {
      this->get_parameter("camera_rate", camConfig_.rate);
      this->get_parameter("camera_range", camConfig_.range);
      this->get_parameter("cam_width", camConfig_.width);
      this->get_parameter("cam_height", camConfig_.height);
      this->get_parameter("cam_fx", camConfig_.fx);
      this->get_parameter("cam_fy", camConfig_.fy);
      this->get_parameter("cam_cx", camConfig_.cx);
      this->get_parameter("cam_cy", camConfig_.cy);
      this->get_parameter("depth_scaling_factor", camConfig_.depth_scaling_factor);
      this->get_parameter("down_sample_factor", down_sample_factor_);
      
      double local_x, local_y, local_z;
      this->get_parameter("local_x", local_x);
      this->get_parameter("local_y", local_y);
      this->get_parameter("local_z", local_z);
      
      gridmap_.setup(res, Eigen::Vector3d(local_x, local_y, local_z), camConfig_.range);
      
      this->get_parameter("depth_filter_tolerance", depth_filter_tolerance_);
      this->get_parameter("depth_filter_mindist", depth_filter_mindist_);
      this->get_parameter("depth_filter_margin", depth_filter_margin_);
      
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      this->get_parameter("p_min", p_min);
      this->get_parameter("p_max", p_max);
      this->get_parameter("p_hit", p_hit);
      this->get_parameter("p_mis", p_mis);
      this->get_parameter("p_occ", p_occ);
      this->get_parameter("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
    }
    gridmap_.inflate_size = inflate_size_;
    
    this->get_parameter("use_mask", use_mask_);

    gridmap_inflate_pub_ = this->create_publisher<quadrotor_msgs::msg::OccMap3d>("gridmap_inflate", 1);
    local_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_pointcloud", 1);
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mask_cloud", 10);

    if (use_global_map_) {
      map_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "global_map", 1, std::bind(&MappingNode::map_call_back, this, std::placeholders::_1));
      global_map_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&MappingNode::global_map_timer_callback, this));
    } else {
      depth_sub_.subscribe(this, "depth", rmw_qos_profile_sensor_data);
      odom_sub_.subscribe(this, "odom", rmw_qos_profile_default); // Odom needs reliability
      
      depth_odom_sync_Ptr_ = std::make_shared<ImageOdomSynchronizer>(
          ImageOdomSyncPolicy(100), depth_sub_, odom_sub_);
      depth_odom_sync_Ptr_->registerCallback(
          std::bind(&MappingNode::depth_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (use_mask_) {
      target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "target", rclcpp::SensorDataQoS(),
          std::bind(&MappingNode::target_odom_callback, this, std::placeholders::_1));
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping

RCLCPP_COMPONENTS_REGISTER_NODE(mapping::MappingNode)
