#ifndef _PHYSICS_CAR_H
#define _PHYSICS_CAR_H 1

#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Physics/RigidBodyDecorator.h>
#include <Physics/CarConfig.h>
#include <list>

namespace OpenEngine {

  namespace Geometry 
    {
      class BoundingGeometry;
    }

  namespace Physics {

    class DynamicBody;

    class Car : public RigidBodyDecorator {

    public:

      /**
       * Create a new instance of a car
       * The chassis should have an AABB as shape.
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

      /**
       * Returns the engine force set on this car.
       * 
       * @return The current engine force.
       */
      float GetEngineForce();

      /**
       * Returns the breaking force set on this car.
       * 
       * @return The current breaking force.
       */
      float GetBrake();

      /**
       * Get the current turn percentage.
       *
       * @return the current turning percentage.
       */
      float GetTurn();

      /**
       * Set the force applied to the engine.
       * The force should be in the range [-1:1]. The real force applied is
       * calculated from the maximum force specified in the CarConfig.
       *
       * @param force The force applied to the engine.
       */
      void SetEngineForce(float force);

      /**
       * Set the force used to break.
       * The force should be in the range [-1:1]. The real force applied is
       * calculated from the maximum breaking force specified in the CarConfig.
       *
       * @param break The force applied to the breaks.
       */
      void SetBrake(float brake);

      /**
       * Specify how much the car should turn. The value should be in the range [-1:1].
       * -1 corresponds to turn full left, while 1 means turn full right. The maximum 
       * angle for turning is determined form the CarConfig.
       *
       * @param trun Specifies how much the wheels should turn.
       */
      void SetTurn(float turn);

      /**
       * The chassis is the rigid body that is controlled by the car physics.
       *
       * @return A rigid body...
       */
      DynamicBody * GetChassis();

      /**
       * When a car is created a CarConfig is initialised with some hopefully
       * reasonable values. The properties of this car can be specified through
       * this CarConfig.
       *
       * @return A reference to a CarConfig object.
       */
      CarConfig & GetConfig();

    private:

      float force;
      float brake;
      float turn;

      CarConfig config;
    };

  }
}
#endif /* _CAR_H */

