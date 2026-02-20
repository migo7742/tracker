#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;

// Ekf 结构体保持不变 (逻辑部分)
struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2; B(1, 1) = t2; B(2, 2) = t2;
    B(3, 0) = dt; B(4, 1) = dt; B(5, 2) = dt;
    C(0, 0) = 1; C(1, 1) = 1; C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4; Qt(1, 1) = 4; Qt(2, 2) = 1;
    Rt(0, 0) = 0.1; Rt(1, 1) = 0.1; Rt(2, 2) = 0.1;
    x.setZero(6);
  }
  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
  }
  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    return (x_tmp.tail(3).norm() <= vmax);
  }
  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }
  inline const Eigen::Vector3d pos() const { return x.head(3); }
  inline const Eigen::Vector3d vel() const { return x.tail(3); }
};

class TargetEkfNode : public rclcpp::Node {
public:
  TargetEkfNode() : Node("target_ekf") {
    // 声明参数
    this->declare_parameter("cam2body_R", std::vector<double>());
    this->declare_parameter("cam2body_p", std::vector<double>());
    this->declare_parameter("cam_fx", 0.0);
    this->declare_parameter("cam_fy", 0.0);
    this->declare_parameter("cam_cx", 0.0);
    this->declare_parameter("cam_cy", 0.0);
    this->declare_parameter("cam_height", 480);
    this->declare_parameter("pitch_thr", 30.0);
    this->declare_parameter("ekf_rate", 20);
    this->declare_parameter("target_class", std::string("person"));
    this->declare_parameter("target_real_height", 1.7);

    // 获取参数
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
    this->get_parameter("cam_height", cam_height_);
    this->get_parameter("pitch_thr", pitch_thr_);
    this->get_parameter("target_class", target_class_);
    this->get_parameter("target_real_height", target_real_height_);

    int ekf_rate = 20;
    this->get_parameter("ekf_rate", ekf_rate);
    ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

    // 初始化时间
    last_update_stamp_ = this->now() - rclcpp::Duration::from_seconds(10.0);

    // Publishers
    target_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("target_odom", 1);
    yolo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("yolo_odom", 1);

    // Subscribers (Standard)
    // 注意: tcpNoDelay 在 ROS2 中通过 QoS Profile 控制，默认 SensorData 类似但不完全相同
    // 这里暂时使用默认 QoS
    single_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 100, std::bind(&TargetEkfNode::odom_callback, this, _1));

    // 单独的 YOLO 订阅用于诊断
    single_yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "yolo", 10, [this](const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
          yolo_count_++;
          if (yolo_count_ % 30 == 1) {
            RCLCPP_INFO(this->get_logger(), "[diag] yolo received (#%d), stamp=%d.%09d, detections=%zu",
                yolo_count_, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->detections.size());
          }
        });

    // Message Filters
    yolo_sub_.subscribe(this, "yolo", rmw_qos_profile_default);
    odom_sub_.subscribe(this, "odom", rmw_qos_profile_default);

    yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(
      YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
    yolo_odom_sync_Ptr_->registerCallback(std::bind(&TargetEkfNode::update_state_callback, this, _1, _2));

    // Timer
    ekf_predict_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / ekf_rate),
      std::bind(&TargetEkfNode::predict_state_callback, this));
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, nav_msgs::msg::Odometry> YoloOdomSyncPolicy;
  typedef message_filters::Synchronizer<YoloOdomSyncPolicy> YoloOdomSynchronizer;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr single_odom_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr single_yolo_sub_;
  int odom_count_ = 0, yolo_count_ = 0, sync_count_ = 0;
  
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  
  rclcpp::TimerBase::SharedPtr ekf_predict_timer_;
  
  std::shared_ptr<Ekf> ekfPtr_;
  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  double fx_, fy_, cx_, cy_;
  int cam_height_;
  double pitch_thr_;
  double target_real_height_;
  std::string target_class_;
  rclcpp::Time last_update_stamp_;
  int consecutive_rejects_ = 0;
  static constexpr int MAX_CONSECUTIVE_REJECTS = 10;

  void predict_state_callback() {
    double update_dt = (this->now() - last_update_stamp_).seconds();
    if (update_dt < 2.0) {
      ekfPtr_->predict();
    } else {
      RCLCPP_WARN(this->get_logger(), "too long time no update!");
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
    target_odom.pose.pose.orientation.w = 1.0;
    target_odom_pub_->publish(target_odom);
  }

  void update_state_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det_msg,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    sync_count_++;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[diag] sync callback #%d! yolo_stamp=%d odom_stamp=%d",
        sync_count_, det_msg->header.stamp.sec, odom_msg->header.stamp.sec);

    Eigen::Vector3d odom_p;
    Eigen::Quaterniond odom_q;
    odom_p(0) = odom_msg->pose.pose.position.x;
    odom_p(1) = odom_msg->pose.pose.position.y;
    odom_p(2) = odom_msg->pose.pose.position.z;
    odom_q.w() = odom_msg->pose.pose.orientation.w;
    odom_q.x() = odom_msg->pose.pose.orientation.x;
    odom_q.y() = odom_msg->pose.pose.orientation.y;
    odom_q.z() = odom_msg->pose.pose.orientation.z;

    Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
    Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

    if (det_msg->detections.empty()) return;

    // 从 Detection2DArray 中选取第一个匹配 target_class_ 的检测
    // 如果 target_class_ 为空则取第一个
    const vision_msgs::msg::Detection2D* chosen = nullptr;
    for (const auto &det : det_msg->detections) {
      if (det.results.empty()) continue;
      if (target_class_.empty() || det.results.front().hypothesis.class_id == target_class_) {
        chosen = &det;
        break;
      }
    }
    if (!chosen) return;

    // center + size → xmin/ymin/xmax/ymax
    double cx = chosen->bbox.center.position.x;
    double cy = chosen->bbox.center.position.y;
    double w  = chosen->bbox.size_x;
    double h  = chosen->bbox.size_y;
    double xmin = cx - w / 2.0;
    double xmax = cx + w / 2.0;
    double ymin = cy - h / 2.0;
    double ymax = cy + h / 2.0;

    if (ymin < pitch_thr_ || ymax > cam_height_ - pitch_thr_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "pitch out of range: ymin=%.0f ymax=%.0f (valid: [%.0f, %d])",
          ymin, ymax, pitch_thr_, static_cast<int>(cam_height_ - pitch_thr_));
      return;
    }

    double height = ymax - ymin;
    double depth = target_real_height_ / height * fy_;
    depth = std::clamp(depth, 0.8, 15.0);
    double y = ((ymin + ymax) * 0.5 - cy_) * depth / fy_;
    double x = ((xmin + xmax) * 0.5 - cx_) * depth / fx_;
    Eigen::Vector3d p(x, y, depth);

    p = cam_q * p + cam_p;

    nav_msgs::msg::Odometry yolo_odom;
    yolo_odom.header.stamp = det_msg->header.stamp;
    yolo_odom.header.frame_id = "odom";
    yolo_odom.pose.pose.orientation.w = 1.0;
    yolo_odom.pose.pose.position.x = p.x();
    yolo_odom.pose.pose.position.y = p.y();
    yolo_odom.pose.pose.position.z = p.z();
    yolo_odom_pub_->publish(yolo_odom);

    double update_dt = (this->now() - last_update_stamp_).seconds();
    if (update_dt > 3.0) {
      ekfPtr_->reset(p);
      consecutive_rejects_ = 0;
      RCLCPP_WARN(this->get_logger(), "ekf reset (stale)!");
    } else if (ekfPtr_->checkValid(p)) {
      ekfPtr_->update(p);
      consecutive_rejects_ = 0;
    } else {
      consecutive_rejects_++;
      if (consecutive_rejects_ >= MAX_CONSECUTIVE_REJECTS) {
        ekfPtr_->reset(p);
        consecutive_rejects_ = 0;
        RCLCPP_WARN(this->get_logger(), "ekf reset (consecutive rejects=%d), pos=(%.2f,%.2f,%.2f)",
            MAX_CONSECUTIVE_REJECTS, p.x(), p.y(), p.z());
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "update invalid! rejects=%d/%d", consecutive_rejects_, MAX_CONSECUTIVE_REJECTS);
      }
      return;
    }
    last_update_stamp_ = this->now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_count_++;
    if (odom_count_ % 100 == 1) {
      RCLCPP_INFO(this->get_logger(), "[diag] odom received (#%d), stamp=%d.%09d",
          odom_count_, msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetEkfNode>());
  rclcpp::shutdown();
  return 0;
}
