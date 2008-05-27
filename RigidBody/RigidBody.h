// RigidBody.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
// 
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------
// Created by christof lemke
// 10 Mar 2008
//--------------------------------------------------------------------

#ifndef _RIGIDBODY_H
#define _RIGIDBODY_H 1

#include "State.h"
#include "Properties.h"

namespace OpenEngine {

  namespace RigidBody {

    /**
     * RigidBody
     *
     * @class RigidBody RigidBody.h RigidBody/RigidBody.h
     */
    template<class Real>
      class RigidBody {

    public:
      /**
       * Create a new instance of a RigidBody
       * 
       * @todo: This constructor is not complete
       * 
       * @param p physical properties of the object
       * @param s initial state
       */
      RigidBody(Properties<Real> p, State<Real> s);

      /**
       * Returns the state that should be used externally
       * for rendering
       *
       * @return the current state of the object
       */
      State<Real> GetCurrentState() const;

      /**
       * Returns the state that should be used internally
       * for computations
       *
       * @todo: restrict access to classes that are allowed to
       * change the state of a rigid body (solver?)
       *
       * @return the working state of the object
       */
      State<Real> & GetWorkingState();

      /**
       * Exchange working and current state
       * 
       * @todo: should be thread safe with accessors
       */
      void SwitchState();

      Properties<Real> properties; //!< Physical properties of object

      /**
       * Compute the derived quantities needed in subsequent steps
       */
      void ComputeDerived();

      /**
       * Computes the force and torque acting on the body.
       */
      void ComputeForceAndTorque(Real t);

      /**
       * Integrates the body's state forward in time using forward Euler.
       */
      void Integrate(Real dt);

    private:
      State<Real> currentState;
      State<Real> workingState;
    };
  }
}

#endif /* _RIGIDBODY_H */
