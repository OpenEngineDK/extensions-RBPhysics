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

#include "RigidBody.h"

#include <cstdlib> //for rand

namespace OpenEngine {

using namespace Math;

  namespace RigidBody {

    template<class Real>
    RigidBody<Real>::RigidBody(Properties<Real> p, State<Real> s) :
      properties(p), currentState(s), workingState(s) {}

    
    template<class Real>
    State<Real> RigidBody<Real>::GetCurrentState() const {
      return currentState;
    }

    template<class Real>
    State<Real> & RigidBody<Real>::GetWorkingState() {
      return workingState;
    }

    template<class Real>
    void RigidBody<Real>::SwitchState() {

    }

    template<class Real>
    void RigidBody<Real>::ComputeDerived() {
      properties.R = workingState.q.GetNormalize().GetMatrix();
      properties.Iinv  = properties.R*properties.Ibodyinv*properties.R.GetTranspose();
      properties.v = workingState.P;
      properties.v /= properties.mass;
      properties.omega = properties.Iinv*workingState.L;
    }

    template<class Real>
    void RigidBody<Real>::ComputeForceAndTorque(Real t) {
      properties.force = Vector<3, Real> (0, 9.82*properties.mass, 0);
      properties.torque = Vector<3, Real> (0,0,0);

      if (t==0) {
	Vector <3, Real> pos = Vector <3, Real> (rand()%10-4.5, rand()%10-4.5, rand()%10-4.5);
	Vector <3, Real> frc = Vector <3, Real> (rand()%10-4.5, -rand()%20-4, 0);
	properties.force += frc;
	properties.torque += (pos-workingState.x)%frc;
      }
    }

    template<class Real>
    void RigidBody<Real>::Integrate(Real dt) {
      workingState.x += properties.v * dt;
      workingState.q += Quaternion<Real>(properties.omega) * workingState.q * dt * 0.5;
      workingState.P += properties.force * dt;
      workingState.L += properties.torque * dt;

    }

    template class RigidBody<float>;
    template class RigidBody<double>;
  }
}
