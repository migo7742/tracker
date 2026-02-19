#ifndef ELLIPSOID_ARRAY_DISPLAY_H
#define ELLIPSOID_ARRAY_DISPLAY_H

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// ROS 2 Headers
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>

// Msg Header
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>

#include "ellipsoid_array_visual.h"

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual;

// ROS 2: 继承自 rviz_common::MessageFilterDisplay
// 消息类型必须是 fully qualified path: decomp_ros_msgs::msg::EllipsoidArray
class EllipsoidArrayDisplay
    : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::EllipsoidArray> {
  Q_OBJECT
public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay();

protected:
  void onInitialize() override; // ROS 2 推荐加 override

  void reset() override;

private Q_SLOTS:
  void updateColorAndAlpha();

private:
  // ROS 2: ConstPtr -> ConstSharedPtr
  void processMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg);

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  // ROS 2: 属性在 rviz_common::properties 命名空间下
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
};
}

#endif
