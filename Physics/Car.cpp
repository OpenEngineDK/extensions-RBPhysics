#include <Physics/Car.h>

#include <Geometry/Box.h>
#include <Geometry/Sphere.h>
#include <Geometry/AABB.h>
#include <Physics/DynamicBody.h>
#include <Geometry/BoundingGeometry.h>

using namespace OpenEngine::Math;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Scene;
using namespace std;

namespace OpenEngine {
  namespace Physics {
    
    Car::Car(DynamicBody * chassis)  :
      RigidBodyDecorator(chassis),
      config(*dynamic_cast<AABB*>(chassis->GetShape()))
    { 
    }

    Car::~Car() {

    }

    Vector<3,float> Car::GetLinearVelocity() const {
      return dynamic_cast<DynamicBody*>(body)->GetLinearVelocity();
    }
    Vector<3,float> Car::GetAngularVelocity() const {
      return dynamic_cast<DynamicBody*>(body)->GetAngularVelocity();
    }
    void Car::SetLinearVelocity(Vector<3,float> vel) {
      dynamic_cast<DynamicBody*>(body)->SetLinearVelocity(vel);
    }
    void Car::SetAngularVelocity(Vector<3,float> vel) {
      dynamic_cast<DynamicBody*>(body)->SetAngularVelocity(vel);
    }

    float Car::GetEngineForce() {
      return force;
    }
    float Car::GetBrake() {
      return brake;
    }
    float Car::GetTurn() {
      return turn;
    }

    void Car::SetEngineForce(float force) {
      this->force = force;
    }
    void Car::SetBrake(float brake) {
      this->brake = brake;
    }
    void Car::SetTurn(float turn) {
      this->turn = turn;
    }

    DynamicBody * Car::GetChassis() {
      return dynamic_cast<DynamicBody*>(body);
    }

    CarConfig & Car::GetConfig() 
    {
      return config;
    }
    
  }
}
