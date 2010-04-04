#ifndef _PHYSICS_IRIGIDBODY_H
#define _PHYSICS_IRIGIDBODY_H 1

#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Geometry/Geometry.h>
#include <string>

namespace OpenEngine {
  namespace Physics {

    class IRigidBody {

    public:

      virtual ~IRigidBody() {};
      /**
       * Get the position of the rigid body calculated by the physics engine.
       *
       * @retrun The position of the rigid body
       */
      virtual OpenEngine::Math::Vector<3,float> GetPosition() = 0;

      /**
       * Get the position of the rigid body calculated by the physics engine.
       *
       * @retrun The rotation of the rigid body
       */
      virtual OpenEngine::Math::Quaternion<float> GetRotation() = 0;

      /**
       * Get the transformation node that can be added to the root
       * of the scene. It will be updated by the physics engine.
       *
       * @return A transformation node.
       */
      virtual OpenEngine::Scene::TransformationNode * GetTransformationNode()= 0;

      /**
       * Set the position of the rigid body.
       * This only affects the physics calculations if it is
       * set before the rigid body is added to the physics engine.
       *
       * @param pos The position of the rigid body
       */
      virtual void SetPosition(OpenEngine::Math::Vector<3,float> pos) = 0;

      /**
       * Set the rotation of the rigid body.
       * This only affects the physics calculations if it is
       * set before the rigid body is added to the physics engine.
       *
       * @param rot The rotation of the rigid body
       */
      virtual void SetRotation(OpenEngine::Math::Quaternion<float> rot) = 0;

      /**
       * Get the shape of the rigid body used for collisions in the physics engine.
       *
       * @retrun The shape of the rigid body
       */
      virtual OpenEngine::Geometry::GeometryBase * GetShape() = 0;

      /**
       * Get the name of the body
       */
      virtual const std::string & GetName() const = 0;
    };

  }
}
#endif

