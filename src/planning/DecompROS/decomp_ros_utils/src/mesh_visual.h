#ifndef MESH_VISUAL_H
#define MESH_VISUAL_H

#include <decomp_geometry/polyhedron.h>
#include <Eigen/Eigenvalues>

#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// MeshShape 移动到了 rviz_rendering
#include <rviz_rendering/objects/mesh_shape.hpp>

namespace decomp_rviz_plugins {
  class MeshVisual {
    public:
      MeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

      virtual ~MeshVisual();

      void setMessage(const vec_E<vec_Vec3f> &bds);
      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      // rviz::MeshShape -> rviz_rendering::MeshShape
      std::unique_ptr<rviz_rendering::MeshShape> obj_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}

#endif
