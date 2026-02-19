#include "ellipsoid_array_visual.h"
// 确保包含 rviz_rendering/objects/shape.hpp

namespace decomp_rviz_plugins {
  EllipsoidArrayVisual::EllipsoidArrayVisual(Ogre::SceneManager *scene_manager,
                                     Ogre::SceneNode *parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
  }

  EllipsoidArrayVisual::~EllipsoidArrayVisual() {
    scene_manager_->destroySceneNode(frame_node_);
  }

  // ROS 2: 参数类型 ConstPtr -> ConstSharedPtr
  void EllipsoidArrayVisual::setMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr &msg) {
    objs_.clear();

    if (msg->ellipsoids.empty())
      return;

    for (const auto& it: msg->ellipsoids) {
      if(std::isnan(it.d[0]) ||
         std::isnan(it.d[1]) ||
         std::isnan(it.d[2]))
        return;
      // 检查 .e 或 .E (取决于消息定义，ROS2 C++ msg struct 成员名通常与 msg 文件中定义一致)
      // 假设 msg 文件里是 E
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          if(std::isnan(it.e[3 * i + j])) // 如果编译报错，尝试改为 it.E
            return;
    }

    objs_.resize(msg->ellipsoids.size());

    for (auto &it : objs_)
      // ROS 2: rviz_rendering::Shape
      it.reset(new rviz_rendering::Shape(rviz_rendering::Shape::Type::Sphere, scene_manager_,
                               frame_node_));

    int cnt = 0;
    for (const auto &it : msg->ellipsoids) {
      Mat3f E_mat;
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          E_mat(i, j) = it.e[3 * i + j]; // 同样注意这里的 E vs e

      Eigen::SelfAdjointEigenSolver<Mat3f> es(E_mat);

      Ogre::Vector3 scale(2 * es.eigenvalues()[0], 2 * es.eigenvalues()[1],
                          2 * es.eigenvalues()[2]);
      objs_[cnt]->setScale(scale);

      Ogre::Vector3 d(it.d[0], it.d[1], it.d[2]);
      objs_[cnt]->setPosition(d);

      Quatf q(es.eigenvectors().determinant() * es.eigenvectors());
      Ogre::Quaternion o(q.w(), q.x(), q.y(), q.z());
      objs_[cnt]->setOrientation(o);
      cnt++;
    }
  }

  void EllipsoidArrayVisual::setFramePosition(const Ogre::Vector3 &position) {
    frame_node_->setPosition(position);
  }

  void EllipsoidArrayVisual::setFrameOrientation(
                                             const Ogre::Quaternion &orientation) {
    frame_node_->setOrientation(orientation);
  }

  void EllipsoidArrayVisual::setColor(float r, float g, float b, float a) {
    for (auto &it : objs_)
      it->setColor(r, g, b, a);
  }
}
