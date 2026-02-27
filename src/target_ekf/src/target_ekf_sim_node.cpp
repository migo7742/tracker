#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 确保此头文件在 include/target_ekf/ 目录下，并且已移除 ROS 1 特定的头文件引用
#include <target_ekf/target_ekf.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class TargetEkfSimNode : public rclcpp::Node {
public:
  TargetEkfSimNode() : Node("target_ekf") {
    // 声明参数
    this->declare_parameter("cam2body_R", std::vector<double>());
    this->declare_parameter("cam2body_p", std::vector<double>());
    this->declare_parameter("cam_fx", 0.0);
    this->declare_parameter("cam_fy", 0.0);
    this->declare_parameter("cam_cx", 0.0);
    this->declare_parameter("cam_cy", 0.0);
    this->declare_parameter("cam_width", 0.0);
    this->declare_parameter("cam_height", 0.0);
    this->declare_parameter("pitch_thr", 30.0);
    this->declare_parameter("check_fov", false);
    this->declare_parameter("ekf_rate", 20);

    // 参数初始化
    last_update_stamp_ = this->now() - rclcpp::Duration::from_seconds(10.0);

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
    
    this->get_parameter("cam_fx", fx_);
    this->get_parameter("cam_fy", fy_);
    this->get_parameter("cam_cx", cx_);
    this->get_parameter("cam_cy", cy_);
    this->get_parameter("cam_width", width_);
    this->get_parameter("cam_height", height_);
    this->get_parameter("pitch_thr", pitch_thr_);
    this->get_parameter("check_fov", check_fov_);

    int ekf_rate = 20;
    this->get_parameter("ekf_rate", ekf_rate);
    ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

    // Publishers
    target_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("target_odom", 1);
    yolo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("yolo_odom", 1);

    // Subscribers
    single_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 100, std::bind(&TargetEkfSimNode::odom_callback, this, _1));

    yolo_sub_.subscribe(this, "yolo", rmw_qos_profile_default);
    odom_sub_.subscribe(this, "odom", rmw_qos_profile_default);

    yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(
        YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
    yolo_odom_sync_Ptr_->registerCallback(std::bind(&TargetEkfSimNode::update_state_callback, this, _1, _2));

    ekf_predict_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ekf_rate),
        std::bind(&TargetEkfSimNode::predict_state_callback, this));
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> YoloOdomSyncPolicy;
  typedef message_filters::Synchronizer<YoloOdomSyncPolicy> YoloOdomSynchronizer;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr single_odom_sub_;
  
  message_filters::Subscriber<nav_msgs::msg::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  
  rclcpp::TimerBase::SharedPtr ekf_predict_timer_;
  
  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  double fx_, fy_, cx_, cy_, width_, height_;
  rclcpp::Time last_update_stamp_;
  double pitch_thr_;
  bool check_fov_;
  std::shared_ptr<Ekf> ekfPtr_;

  void predict_state_callback() {
    double update_dt = (this->now() - last_update_stamp_).seconds();
    if (update_dt < 2.0) {
      ekfPtr_->predict();
    } else {
      RCLCPP_WARN(this->get_logger(), "[ekf] too long time no update!");
      return;
    }
    
    nav_msgs::msg::Odometry target_odom;
    target_odom.header.stamp = this->now();
    target_odom.header.frame_id = "odom";
    target_odom.pose.pose.position.x = ekfPtr_->pos().x();
    target_odom.pose.pose.position.y = ekfPtr_->pos().y();
    target_odom.pose.pose.position.z = ekfPtr_->pos().z();
    target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
    target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
    target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
    
    Eigen::Vector3d rpy = ekfPtr_->rpy();
    // 假设 euler2quaternion 在 target_ekf.hpp 中定义，并且返回 Eigen::Quaterniond
    Eigen::Quaterniond q = euler2quaternion(rpy); 
    
    target_odom.pose.pose.orientation.w = q.w();
    target_odom.pose.pose.orientation.x = q.x();
    target_odom.pose.pose.orientation.y = q.y();
    target_odom.pose.pose.orientation.z = q.z();
    target_odom_pub_->publish(target_odom);
  }

  void update_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& target_msg, 
                             const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    Eigen::Vector3d odom_p, p;
    Eigen::Quaterniond odom_q, q;
    
    odom_p(0) = odom_msg->pose.pose.position.x;
    odom_p(1) = odom_msg->pose.pose.position.y;
    odom_p(2) = odom_msg->pose.pose.position.z;
    odom_q.w() = odom_msg->pose.pose.orientation.w;
    odom_q.x() = odom_msg->pose.pose.orientation.x;
    odom_q.y() = odom_msg->pose.pose.orientation.y;
    odom_q.z() = odom_msg->pose.pose.orientation.z;

    Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
    Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

    p.x() = target_msg->pose.pose.position.x;
    p.y() = target_msg->pose.pose.position.y;
    p.z() = target_msg->pose.pose.position.z;
    q.w() = target_msg->pose.pose.orientation.w;
    q.x() = target_msg->pose.pose.orientation.x;
    q.y() = target_msg->pose.pose.orientation.y;
    q.z() = target_msg->pose.pose.orientation.z;

    // 假设 quaternion2euler 在 target_ekf.hpp 中定义
    Eigen::Vector3d rpy = quaternion2euler(q);

    if (check_fov_) {
      Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
      if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) return;
      
      double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
      if (x < 0 || x > height_) return;
      
      double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
      if (y < 0 || y > width_) return;
    }

    double update_dt = (this->now() - last_update_stamp_).seconds();
    if (update_dt > 5.0) {
      ekfPtr_->reset(p, rpy);
      RCLCPP_WARN(this->get_logger(), "[ekf] reset!");
    } else if (ekfPtr_->update(p, rpy)) {
      // Update success
    } else {
      RCLCPP_ERROR(this->get_logger(), "[ekf] update invalid!");
      return;
    }
    last_update_stamp_ = this->now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    (void)msg;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetEkfSimNode>());
  rclcpp::shutdown();
  return 0;
}
