#ifndef _PHYSICS_RIGIDBODYDECORATOR_H
#define _PHYSICS_RIGIDBODYDECORATOR_H 1

#include <Physics/IRigidBody.h>
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Geometry/Geometry.h>

namespace OpenEngine {
  namespace Physics {

    class RigidBodyDecorator : public IRigidBody {

    protected:
      IRigidBody * body;

    public:
      RigidBodyDecorator(IRigidBody * body);

      // Inherited form IRigidBody:
      virtual ~RigidBodyDecorator();
      OpenEngine::Math::Vector<3,float> GetPosition();
      OpenEngine::Math::Quaternion<float> GetRotation();
      OpenEngine::Scene::TransformationNode * GetTransformationNode();
      void SetPosition(OpenEngine::Math::Vector<3,float> pos);
      void SetRotation(OpenEngine::Math::Quaternion<float> rot);
      OpenEngine::Geometry::Geometry * GetShape();
      const std::string & GetName() const;
      void SetName(const std::string & name);

    private:
      std::string name;
    };

  }
}
#endif

