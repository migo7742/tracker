#include <Eigen/Geometry>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std;

class PathVisualizer : public rclcpp::Node {
public:
  PathVisualizer() : Node("visualize_history_path") {
    RCLCPP_INFO(this->get_logger(), "visualize_history_path node init ok");

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "/position_cmd", 100, std::bind(&PathVisualizer::cmd_callback, this, std::placeholders::_1));
        
    target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/object_odom_dtc2brig", 100, std::bind(&PathVisualizer::target_odom_callback, this, std::placeholders::_1));

    drone_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("history_drone_pose", 100);
    target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("history_target_pose", 100);
    lines_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_target_link_line", 100);
    spring_pub_ = this->create_publisher<nav_msgs::msg::Path>("spring", 100);

    t_last_ = this->now();
  }

private:
  nav_msgs::msg::Path drone_path, target_path;
  visualization_msgs::msg::Marker line_list;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr drone_path_pub_, target_path_pub_, spring_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lines_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
  
  rclcpp::Time t_last_;

  void target_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    target_path.header.frame_id = "world";
    target_path.header.stamp = this->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = msg->pose.pose.position.y;
    pose.pose.position.z = msg->pose.pose.position.z;
    target_path.poses.push_back(pose);
    target_path_pub_->publish(target_path);
    // RCLCPP_INFO(this->get_logger(), "publish path once");

    // NOTE spring
    if (drone_path.poses.empty()) {
      return;
    }

    {
      Eigen::Vector3d p0, p1;
      p0.x() = msg->pose.pose.position.x;
      p0.y() = msg->pose.pose.position.y;
      p0.z() = msg->pose.pose.position.z;

      p1.x() = drone_path.poses.back().pose.position.x;
      p1.y() = drone_path.poses.back().pose.position.y;
      p1.z() = drone_path.poses.back().pose.position.z;

      Eigen::Vector3d dp = p1 - p0;
      Eigen::Vector3d dx = dp.normalized();
      // Handle the case where dx is parallel to Z (0,0,1)
      Eigen::Vector3d unit_z(0, 0, 1);
      Eigen::Vector3d dy;
      if (std::abs(dx.dot(unit_z)) > 0.99) {
          dy = Eigen::Vector3d(1, 0, 0).cross(dx);
      } else {
          dy = unit_z.cross(dx);
      }
      Eigen::Vector3d dz = dx.cross(dy);

      nav_msgs::msg::Path spring_msg;
      spring_msg.header.frame_id = "world";
      spring_msg.header.stamp = this->now();
      
      for (double t = 0; t < 10 * 2 * M_PI; t += 0.1) {
        double y = 0.2 * cos(t);
        double z = 0.2 * sin(t);
        Eigen::Vector3d p = p0 + dp * t / (10 * 2 * M_PI) + z * dz + y * dy;
        geometry_msgs::msg::PoseStamped pose_spring;
        pose_spring.pose.position.x = p.x();
        pose_spring.pose.position.y = p.y();
        pose_spring.pose.position.z = p.z();
        spring_msg.poses.push_back(pose_spring);
      }
      spring_pub_->publish(spring_msg);
    }

    rclcpp::Time t_now = this->now();
    if ((t_now - t_last_).seconds() < 0.3) {
      return;
    }
    t_last_ = t_now;
    
    line_list.header.stamp = t_now;
    line_list.header.frame_id = "world";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.ns = "lines";
    line_list.id = 0; // Set ID
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.03;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.1;
    line_list.color.a = 0.5;

    geometry_msgs::msg::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    line_list.points.push_back(p);

    p.x = drone_path.poses.back().pose.position.x;
    p.y = drone_path.poses.back().pose.position.y;
    p.z = drone_path.poses.back().pose.position.z;
    line_list.points.push_back(p);

    lines_pub_->publish(line_list);
  }

  void cmd_callback(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
    drone_path.header.frame_id = "world";
    drone_path.header.stamp = this->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = cmd->position.x;
    pose.pose.position.y = cmd->position.y;
    pose.pose.position.z = cmd->position.z;
    drone_path.poses.push_back(pose);
    drone_path_pub_->publish(drone_path);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathVisualizer>());
  rclcpp::shutdown();
  return 0;
}
