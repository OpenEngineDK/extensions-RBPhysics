#ifndef _PHYSICS_DYNAMICBODY_H
#define _PHYSICS_DYNAMICBODY_H 1

#include "RigidBodyDecorator.h"
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Scene/TransformationNode.h>
#include <Geometry/Geometry.h>
#include <list>
#include <boost/tuple/tuple.hpp>

namespace OpenEngine {
  namespace Physics {

    /**
     * Representation of a rigid body that can be affected
     * by forces from outside the physics engine.
     */
    class DynamicBody : public RigidBodyDecorator {

    public:

      /**
       * Create a new instance of a rigid body
       *
       * @param transNode TransformationNode controled by the physics engine
       * @param shape The shape of the rigid body used for collisions
       */
      DynamicBody(IRigidBody * body);

      ~DynamicBody();

      /**
       * Set the position of the rigid body.
       * This change is propagated to the physics engine.
       *
       * @param pos The new position of the rigid body
       */
      void SetPosition(OpenEngine::Math::Vector<3,float> pos);

      /**
       * Set the rotation of the rigid body.
       * This change is propagated to the physics engine.
       *
       * @param rot The new rotation of the rigid body
       */
      void SetRotation(OpenEngine::Math::Quaternion<float> rot);
      
      /**
       * Get the linear velocity of the rigid body calculated by the physics engine.
       *
       * @return The linear velocity of the rigid body
       */
      OpenEngine::Math::Vector<3,float> GetLinearVelocity() const;

      /**
       * Get the angular velocity of the rigid body calculated by the physics engine.
       *
       * @return The angular velocity of the rigid body
       */
      OpenEngine::Math::Vector<3,float> GetAngularVelocity() const;

      typedef boost::tuple< const OpenEngine::Math::Vector<3,float>, const OpenEngine::Math::Vector<3,float> > ForceAtPosition;

      /**
       * Get the mass of the rigid body used for dynamics in the physics engine.
       *
       * @return The mass of the rigid body
       */
      float GetMass() const;

      /**
       * Set the linear velocity of the rigid body.
       * This only affects the physics calculations if it is
       * set before the rigid body is added to the physics engine.
       *
       * @param vel The linear velocity of the rigid body
       */
      void SetLinearVelocity(OpenEngine::Math::Vector<3,float>);

      /**
       * Set the angular velocity of the rigid body.
       * This only affects the physics calculations if it is
       * set before the rigid body is added to the physics engine.
       *
       * @param vel The angular velocity of the rigid body
       */
      void SetAngularVelocity(OpenEngine::Math::Vector<3,float> vel);

      /**
       * Set the mass of the rigid body.
       * This only affects the physics calculations if it is
       * set before the rigid body is added to the physics engine.
       *
       * @param mass The mass of the rigid body
       */
      void SetMass(float mass);

      /**
       * Apply a force at a spcific position on the rigid body.
       * This will change the linear and angular velocity of
       * the rigid body.
       * Use this method to affect the body from the outside.
       *
       * @param force The force applied to the body.
       * @param position The point were the force is applied.
       */
      void ApplyForce(OpenEngine::Math::Vector<3,float> force, 
                      OpenEngine::Math::Vector<3,float> position);

      /**
       * Apply a force at center of the rigid body.
       * This will change the linear velocity of
       * the rigid body.
       * Use this method to affect the body from the outside.
       *
       * @param force The force applied to the body.
       */
      void ApplyForce(OpenEngine::Math::Vector<3,float> force);

      /**
       * Apply a torque on the rigid body.
       * This will change the angular velocity of
       * the rigid body.
       * Use this method to affect the body from the outside.
       *
       * @param torque The torque applied to the body.
       */
      void ApplyTorque(OpenEngine::Math::Vector<3,float> torque);

      /**
       * Returns a list of tuples of forces and points that will
       * affect the body in the next physics step.
       * This list is reset after every simulation step.
       *
       * @return A list of forces and points.
       */
      std::list<ForceAtPosition> & GetForces();
      
      /**
       * Returns the torque that will
       * affect the body in the next physics step.
       * This value is reset after every simulation step.
       *
       * @return The torque affecting the body
       */
      const OpenEngine::Math::Vector<3,float> & GetTorque() const;

      /**
       * Reset the forces and the torque.
       * Is used internally in the physics engine.
       * It should not be necessary to call this from
       * outside
       */
      void ResetForces();

      
      /**
       * Set the damping of the linear velocity
       * of the rigid body.
       * This change is propagated to the physics engine.
       * This method is intended to be used before the
       * object is inserted in the physics engine.
       *
       * @param pos The new position of the rigid body
       */
      void SetLinearDamping(float damp);
      /**
       * Set the damping of the angular velocity
       * of the rigid body.
       * This change is propagated to the physics engine.
       * This method is intended to be used before the
       * object is inserted in the physics engine.
       *
       * @param pos The new position of the rigid body
       */
      void SetAngularDamping(float damp);

      /**
       * Get the linear damping of the rigid body.
       *
       * @return the linear damping.
       */
      float GetLinearDamping() const;

      /**
       * Get the angular damping of the rigid body.
       *
       * @return the linear damping.
       */
      float GetAngularDamping() const;

      /**
       * Toggle deactivation state of the body.
       * Deactivation should be disabled if the body
       * will be affected by forces. A disabled body will still
       * participate in collisions.
       * Bodies will be deactivated when they are not moving.
       * 
       * @param deactivate If this is set to true, deactivation is disabled.
       */
      void SetDisableDeactivation(bool deactivate);

      /**
       * Is Deactivation enabled for this body.
       */
      bool IsDisableDeactivation();

      /**
       * Check if the state of the object has been change
       * from outside the physics engine.
       * This method will return true if the position,
       * rotation or linear/angular velocity were changed.
       *
       * @return true if the state has changed.
       */
      bool IsStateChanged() const;

      /**
       * Used internally by the physics engine to reset
       * the stateChanged variable.
       *
       * @param state set to false to signal that this body is in sync with its internal representation.
       */
      void SetStateChanged(bool state);

    private:

      OpenEngine::Math::Vector<3,float> linearVelocity;
      OpenEngine::Math::Vector<3,float> angularVelocity;

      float linearDamping;
      float angularDamping;

      OpenEngine::Scene::TransformationNode transNode;
      std::list<ForceAtPosition> forces;
      OpenEngine::Math::Vector<3,float> torque;
      
      float mass;

      bool deactivation;

      bool stateChanged;
    };

  }
}
#endif /* _RIGIDBODY_H */

