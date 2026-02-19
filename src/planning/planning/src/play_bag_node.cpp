#include <mapping/mapping.h>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <quadrotor_msgs/msg/replan_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <wr_msg/wr_msg.hpp>

// Use a class to hold the node state or global node pointer if strictly necessary
// For simplicity in this small node, we can use a global node pointer for logging inside callback
// or capture it in a lambda. Here we use a standard class approach.

class PlayBagNode : public rclcpp::Node {
public:
  PlayBagNode() : Node("play_bag_node") {
    // Use Reliable QoS for state data to ensure we don't miss the trigger
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    replanState_sub_ = this->create_subscription<quadrotor_msgs::msg::ReplanState>(
        "replanState", qos, 
        std::bind(&PlayBagNode::replan_state_callback, this, std::placeholders::_1));
  }

private:
  void replan_state_callback(const quadrotor_msgs::msg::ReplanState::SharedPtr msgPtr) {
    // RCLCPP_WARN(this->get_logger(), "[log] REPLAN STATE RECEIVED!");
    if (msgPtr->state == 2) {
      std::string package_path = ament_index_cpp::get_package_share_directory("planning");
      // Note: In ROS 2, packages are installed to install/share/<package_name>.
      // The relative path logic "/../../../" might need adjustment depending on where 'debug' folder is expected.
      // Assuming the user wants to go up from share/planning -> install/share -> install -> workspace_root -> debug?
      // You might need to adjust this path manually.
      std::string file_path = package_path + "/../../../debug/replan_state.bin";
      
      RCLCPP_INFO(this->get_logger(), "Writing replan state to: %s", file_path.c_str());
      
      wr_msg::writeMsg(*msgPtr, file_path);
      
      // Force crash/stop as per original logic
      assert(false);
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<quadrotor_msgs::msg::ReplanState>::SharedPtr replanState_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlayBagNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
