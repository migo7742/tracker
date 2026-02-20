#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <prediction/prediction.hpp>
#include <visualization/visualization.hpp>

#include <rclcpp/rclcpp.hpp>
#include <memory>

class TestNode : public rclcpp::Node {
 public:
  TestNode() : Node("test_node") {
    auto node_ptr = this->shared_from_this();
    prePtr_ = std::make_shared<prediction::Predict>(node_ptr);
    visPtr_ = std::make_shared<visualization::Visualization>(node_ptr);

    gridmap_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
        "~/gridmap_inflate", 1,
        std::bind(&TestNode::gridmap_callback, this, std::placeholders::_1));

    triger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/triger", 10,
        std::bind(&TestNode::triger_callback, this, std::placeholders::_1));

    test_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        std::bind(&TestNode::testCallback, this));

    RCLCPP_WARN(this->get_logger(), "[TEST NODE]: ready.");
  }

 private:
  std::shared_ptr<prediction::Predict> prePtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  mapping::OccGridMap map_;
  bool map_received_ = false;

  Eigen::Vector3d target_p_, target_v_;

  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr triger_sub_;
  rclcpp::TimerBase::SharedPtr test_timer_;

  void gridmap_callback(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr& msgPtr) {
    if (map_received_) {
      return;
    }
    map_.from_msg(*msgPtr);
    prePtr_->setMap(map_);
    RCLCPP_WARN(this->get_logger(), "[TEST NODE] GLOBAL MAP RECEIVED");
    map_received_ = true;
  }

  void testCallback() {
  }

  void triger_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msgPtr) {
    target_p_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    target_v_ = Eigen::Vector3d(1, 0, 0);

    std::vector<Eigen::Vector3d> predict_path;
    std::cout << "target_p: " << target_p_.transpose() << std::endl;
    std::cout << "target_v: " << target_v_.transpose() << std::endl;
    prePtr_->predict(target_p_, target_v_, predict_path);
    visPtr_->visualize_pointcloud(predict_path, "future_pts");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
