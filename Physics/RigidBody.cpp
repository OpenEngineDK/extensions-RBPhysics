#include <Physics/RigidBody.h>

#include <Geometry/Box.h>
#include <Geometry/Sphere.h>
#include <Geometry/TriangleMesh.h>

using namespace OpenEngine::Math;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Scene;
using namespace std;

namespace OpenEngine {
  namespace Physics {
    
    RigidBody::RigidBody(OpenEngine::Geometry::GeometryBase * shape) :
      transNode(),
      shape(shape),
      name("")
    {
    }

    RigidBody::~RigidBody()
	{
    }

    Vector<3,float> RigidBody::GetPosition() {
      return transNode.GetPosition();
    }

    Quaternion<float> RigidBody::GetRotation() {
      return transNode.GetRotation();
    }
    
    OpenEngine::Geometry::GeometryBase * RigidBody::GetShape() {
      return shape;
    }    
    
    OpenEngine::Scene::TransformationNode * RigidBody::GetTransformationNode() {
      return &transNode;
    }
    
    
    void RigidBody::SetPosition(Vector<3,float> pos) {
      transNode.SetPosition(pos);
    }
    
    void RigidBody::SetRotation(Quaternion<float> rot) {
      transNode.SetRotation(rot);
    }

    const string & RigidBody::GetName() const {
      return name;
    }

    void RigidBody::SetName(const string & name) {
      this->name = name;
    }
  }
}
