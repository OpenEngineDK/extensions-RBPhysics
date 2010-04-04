#ifndef _PHYSICS_RIGIDBODY_H
#define _PHYSICS_RIGIDBODY_H 1

#include <Physics/IRigidBody.h>
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Geometry/Geometry.h>
#include <list>

namespace OpenEngine {
  namespace Physics {

    /**
     * Representation of a static rigid body.
     */
    class RigidBody : public IRigidBody {

    public:

      /**
       * Create a new instance of a rigid body
       *
       * @param transNode TransformationNode controled by the physics engine
       * @param shape The shape of the rigid body used for collisions
       */
      RigidBody(OpenEngine::Geometry::GeometryBase * shape);

      ~RigidBody();

      // Inherited from IRigidBody:
      OpenEngine::Math::Vector<3,float> GetPosition();
      OpenEngine::Math::Quaternion<float> GetRotation();
      OpenEngine::Geometry::GeometryBase * GetShape();
      void SetPosition(OpenEngine::Math::Vector<3,float> pos);
      void SetRotation(OpenEngine::Math::Quaternion<float> rot);
      OpenEngine::Scene::TransformationNode * GetTransformationNode();
      const std::string & GetName() const;
      void SetName(const std::string & name);

    private:
      OpenEngine::Scene::TransformationNode transNode;
      OpenEngine::Geometry::GeometryBase * shape;
      std::string name;
    };

  }
}
#endif /* _RIGIDBODY_H */

