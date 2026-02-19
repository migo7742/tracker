// ROS 2 headers
#include <rviz_common/logging.hpp> 
#include <pluginlib/class_list_macros.hpp> 

#include "ellipsoid_array_display.h"

namespace decomp_rviz_plugins {

EllipsoidArrayDisplay::EllipsoidArrayDisplay() {
  // ROS 2: ColorProperty 位于 rviz_common::properties
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(204, 51, 204),
      "Color of ellipsoids.",
      this, SLOT(updateColorAndAlpha()));
      
  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
}

void EllipsoidArrayDisplay::onInitialize() {
  MFDClass::onInitialize();
}

EllipsoidArrayDisplay::~EllipsoidArrayDisplay() {}

void EllipsoidArrayDisplay::reset() {
  MFDClass::reset();
  visual_ = nullptr;
}

void EllipsoidArrayDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  if (visual_)
    visual_->setColor(color.r, color.g, color.b, alpha);
}

// ROS 2: 参数改为 ConstSharedPtr
void EllipsoidArrayDisplay::processMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // ROS 2: getTransform 签名改变。
  // 1. context_ 是 rviz_common::DisplayContext*
  // 2. getFrameManager() 返回 FrameManagerIface*
  // 3. header.stamp 是 builtin_interfaces::msg::Time，需要转为 rclcpp::Time
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, rclcpp::Time(msg->header.stamp), position, orientation)) {
    // ROS 2 Logging
    RCLCPP_DEBUG(rclcpp::get_logger("EllipsoidArrayDisplay"), 
              "Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  std::shared_ptr<EllipsoidArrayVisual> visual;
  visual.reset(new EllipsoidArrayVisual(context_->getSceneManager(), scene_node_));

  // 注意：visual->setMessage 需要接受 ConstSharedPtr 或者 raw pointer，需检查 visual 的定义
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

}

// ROS 2: Plugin 导出宏
PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::EllipsoidArrayDisplay, rviz_common::Display)
