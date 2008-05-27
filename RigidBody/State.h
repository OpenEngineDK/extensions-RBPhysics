// State.
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

#ifndef _STATE_H
#define _STATE_H 1

#include <Math/Vector.h>
#include <Math/Quaternion.h>

namespace OpenEngine {

  namespace RigidBody {

    /**
     * The State of a RigidBody. Defining Position, Rotation, Linear and Angular Momentum.
     *
     * @class State State.h RigidBody/State.h
     */
    template <class Real>
      class State {

    public:
      
      /**
       * Create new instance of a rigid body state
       * 
       * @param x position
       * @param q rotation
       */
    State(Math::Vector<3,Real> x, Math::Quaternion<Real> q) :
      x(x), q(q) {}

      Math::Vector<3,Real> x; //!< Position
      Math::Quaternion<Real> q; //!< Rotation
      Math::Vector<3,Real> P; //!< Linear Momentum
      Math::Vector<3,Real> L; //!< Angular Momentum
    private:

    };

  }}

#endif /* _STATE_H */

