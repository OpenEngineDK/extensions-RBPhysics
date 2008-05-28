#ifndef _PHYSICS_CAR_H
#define _PHYSICS_CAR_H 1

#include "RigidBodyDecorator.h"
#include "DynamicBody.h"
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Geometry/BoundingGeometry.h>
#include <list>

namespace OpenEngine {
  namespace Physics {

    class Car : public RigidBodyDecorator {

    public:

      /**
       * Create a new instance of a car
       *
       * @param chassis The body of the car
       */
      Car(DynamicBody * chassis);

      ~Car();

      /**
       * Get the linear velocity of the car calculated by the physics engine.
       *
       * @return The linear velocity of the car
       */
      OpenEngine::Math::Vector<3,float> GetLinearVelocity() const;
      /**
       * Get the angular velocity of the car calculated by the physics engine.
       *
       * @return The angular velocity of the car
       */
      OpenEngine::Math::Vector<3,float> GetAngularVelocity() const;

      /**
       * Set the linear velocity of the car.
       * This only affects the physics calculations if it is
       * set before the car is added to the physics engine.
       *
       * @param vel The linear velocity of the car
       */
      void SetLinearVelocity(OpenEngine::Math::Vector<3,float>);
      /**
       * Set the angular velocity of the car.
       * This only affects the physics calculations if it is
       * set before the car is added to the physics engine.
       *
       * @param vel The angular velocity of the car
       */
      void SetAngularVelocity(OpenEngine::Math::Vector<3,float> vel);

      void Accelerate(float accel);
      void Brake(float brake);
      void Turn(float turn);

      DynamicBody * GetChassis();
    };

  }
}
#endif /* _CAR_H */

