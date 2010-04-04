#include <Physics/RigidBodyDecorator.h>

using namespace std;

namespace OpenEngine {
  namespace Physics {

    RigidBodyDecorator::RigidBodyDecorator(IRigidBody * body) :
      body(body) 
    {
    }
    
    RigidBodyDecorator::~RigidBodyDecorator() {

    }

    OpenEngine::Math::Vector<3,float> RigidBodyDecorator::GetPosition() 
    {
      return body->GetPosition();
    }
    
    OpenEngine::Math::Quaternion<float> RigidBodyDecorator::GetRotation() 
    {
      return body->GetRotation();
    }
    
    OpenEngine::Scene::TransformationNode * RigidBodyDecorator::GetTransformationNode() 
    {
      return body->GetTransformationNode();
    }
    
    void RigidBodyDecorator::SetPosition(OpenEngine::Math::Vector<3,float> pos) 
    {
      body->SetPosition(pos);
    }
    
    void RigidBodyDecorator::SetRotation(OpenEngine::Math::Quaternion<float> rot) 
    {
      body->SetRotation(rot);
    }
    
    OpenEngine::Geometry::GeometryBase * RigidBodyDecorator::GetShape() 
    {
      return body->GetShape();
    }

    const string & RigidBodyDecorator::GetName() const {
      return name;
    }

    void RigidBodyDecorator::SetName(const string & name) {
      this->name = name;
    }
  }
}
