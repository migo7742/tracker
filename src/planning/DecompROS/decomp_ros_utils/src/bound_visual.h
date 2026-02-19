#ifndef BOUND_VISUAL_H
#define BOUND_VISUAL_H

#include <decomp_basis/data_type.h>

#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// ROS 2: BillboardLine 移动到了 rviz_rendering
#include <rviz_rendering/objects/billboard_line.hpp>

namespace decomp_rviz_plugins {
class BoundVisual {
public:
  BoundVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  ~BoundVisual();

  void setMessage(const vec_E<vec_Vec3f> &bds);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  // ROS 2: 命名空间变更为 rviz_rendering
  std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> objs_;

  Ogre::SceneNode *frame_node_;

  Ogre::SceneManager *scene_manager_;
};
}

#endif
