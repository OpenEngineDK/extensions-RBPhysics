#include "Car.h"

#include <Geometry/Box.h>
#include <Geometry/Sphere.h>

using namespace OpenEngine::Math;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Scene;
using namespace std;

namespace OpenEngine {
  namespace Physics {
    
    Car::Car(DynamicBody * chassis)  :
      RigidBodyDecorator(body) {

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
  }
}
