#ifndef ELLIPSOIDS_VISUAL_H
#define ELLIPSOIDS_VISUAL_H

// ROS 2 消息头文件
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_geometry/ellipsoid.h>
#include <Eigen/Eigenvalues>

// Ogre 头文件 (ROS 2 通常直接包含文件名)
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// Rviz Rendering Objects
#include <rviz_rendering/objects/shape.hpp>

namespace decomp_rviz_plugins {
  class EllipsoidArrayVisual {
    public:
      EllipsoidArrayVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node);

      virtual ~EllipsoidArrayVisual();

      // ConstPtr -> ConstSharedPtr
      void setMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr &msg);

      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      // rviz::Shape -> rviz_rendering::Shape
      std::vector<std::unique_ptr<rviz_rendering::Shape>> objs_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}
#endif
