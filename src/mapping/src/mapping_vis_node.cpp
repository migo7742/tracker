#include "mapping/mapping.h"
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MappingVisNode : public rclcpp::Node {
public:
  MappingVisNode() : Node("mapping_vis") {
    this->declare_parameter("remove_floor_ceil", false);
    this->get_parameter("remove_floor_ceil", remove_floor_ceil_);

    gridmap_vs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap", 1);
    gridmap_inflate_vs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap_inflate", 1);

    gridmap_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
        "gridmap", 1, std::bind(&MappingVisNode::gridmap_callback, this, std::placeholders::_1));
    
    gridmap_inflate_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
        "gridmap_inflate", 1, std::bind(&MappingVisNode::gridmap_inflate_callback, this, std::placeholders::_1));
  }

private:
  void gridmap_callback(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msgPtr) {
    mapping::OccGridMap gridmap;
    gridmap.from_msg(*msgPtr);
    sensor_msgs::msg::PointCloud2 pc;
    gridmap.occ2pc(pc);
    pc.header.frame_id = "odom";
    pc.header.stamp = this->now();
    gridmap_vs_pub_->publish(pc);
  }

  void gridmap_inflate_callback(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msgPtr) {
    mapping::OccGridMap gridmap;
    gridmap.from_msg(*msgPtr);
    sensor_msgs::msg::PointCloud2 pc;
    if (remove_floor_ceil_) {
      gridmap.occ2pc(pc, 0.5, 2.5);
    } else {
      gridmap.occ2pc(pc);
    }
    pc.header.frame_id = "odom";
    pc.header.stamp = this->now();
    gridmap_inflate_vs_pub_->publish(pc);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gridmap_vs_pub_, gridmap_inflate_vs_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_, gridmap_inflate_sub_;
  bool remove_floor_ceil_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingVisNode>());
  rclcpp::shutdown();
  return 0;
}
