#include <Physics/DynamicBody.h>

#include <Geometry/Box.h>
#include <Geometry/Sphere.h>
#include <Geometry/TriangleMesh.h>
#include <Geometry/HeightfieldTerrainShape.h>
#include <Core/Exceptions.h>

using namespace OpenEngine::Math;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Scene;
using namespace OpenEngine::Core;

using namespace std;

namespace OpenEngine {
  namespace Physics {
    
    DynamicBody::DynamicBody(IRigidBody * body) :
      RigidBodyDecorator(body),
      linearVelocity(Vector<3,float>(0.0)),
      angularVelocity(Vector<3,float>(0.0)),
      linearDamping(0.0f),
      angularDamping(0.0f),
      transNode(),
      mass(1.0),
      deactivation(false),
      stateChanged(true)
    {
        /*
      if(typeid(TriangleMesh) == typeid(*this->body->GetShape()) ||
         typeid(HeightfieldTerrainShape) == typeid(*this->body->GetShape())) {
        throw InvalidArgument("dynamic concave bodies not supported");
      }
      */
    }

    DynamicBody::~DynamicBody() {

    }

    void DynamicBody::SetPosition(OpenEngine::Math::Vector<3,float> pos) {
      stateChanged = true;
      body->SetPosition(pos);
    }

    void DynamicBody::SetRotation(OpenEngine::Math::Quaternion<float> rot) {
      stateChanged = true;
      body->SetRotation(rot);
    }
    
    Vector<3,float> DynamicBody::GetLinearVelocity() const {
      return linearVelocity;
    }

    Vector<3,float> DynamicBody::GetAngularVelocity() const {
      return angularVelocity;
    }

    float DynamicBody::GetMass() const {
      return mass;
    }
    
    void DynamicBody::SetLinearVelocity(Vector<3,float> vel) {
      stateChanged = true;
      linearVelocity = vel;
    }
    
    void DynamicBody::SetAngularVelocity(Vector<3,float> vel) {
      stateChanged = true;
      angularVelocity = vel;
    }
    
    void DynamicBody::SetMass(float mass) {
      this->mass = mass;
    }

    
    void DynamicBody::ApplyForce(Vector<3,float> force, Vector<3,float> position) 
    {
      forces.push_back(ForceAtPosition(force,position));
    }

    void DynamicBody::ApplyForce(Vector<3,float> force) 
    {
      forces.push_back(ForceAtPosition(force,Vector<3,float>(0,0,0)));
    }
    
    void DynamicBody::ApplyTorque(Vector<3,float> torque) 
    {
      this->torque += torque;
    }
    

    std::list<DynamicBody::ForceAtPosition> & DynamicBody::GetForces() {
      return forces;
    }
    
    const Vector<3,float> & DynamicBody::GetTorque() const
    {
      return torque;
    }

    
    void DynamicBody::ResetForces() 
    {
      torque = Vector<3,float>();
      forces.clear();
    }

    void DynamicBody::SetLinearDamping(float damp) {
      this->stateChanged = true;
      
      this->linearDamping = damp;
      
      if(linearDamping < 0.0f) {
        linearDamping = 0.0f;
      }
    }

    void DynamicBody::SetAngularDamping(float damp) {
      this->stateChanged = true;
      
      this->angularDamping = damp;

      if(angularDamping < 0.0f) {
        angularDamping = 0.0f;
      }
    }

    float DynamicBody::GetLinearDamping() const {
      return linearDamping;
    }

    float DynamicBody::GetAngularDamping() const {
      return angularDamping;
    }

    void DynamicBody::SetDisableDeactivation(bool deactivate) 
    {
      this->stateChanged = true;
      this->deactivation = deactivate;
    }
    

    bool DynamicBody::IsDisableDeactivation() 
    {
      return this->deactivation;
    }

    bool DynamicBody::IsStateChanged() const {
      return stateChanged;
    }

    void DynamicBody::SetStateChanged(bool state) {
      this->stateChanged = state;
    }
  }
}
