// 独立可执行入口，用于配合 AddressSanitizer 检测内存错误
// 直接包含 planning_nodelet.cpp 以获取 PlanningNode 类定义
#include "planning_nodelet.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<planning::PlanningNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
