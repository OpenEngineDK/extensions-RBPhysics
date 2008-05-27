// Properties.
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

#include "Properties.h"

namespace OpenEngine {

  using namespace Math;

  namespace RigidBody {

    template<class Real>
    Properties<Real>::Properties(const Real mass,
				 const Matrix<3,3,Real> Ibody,
				 const Matrix<3,3,Real> Ibodyinv) :
      mass(mass), Ibody(Ibody), Ibodyinv(Ibodyinv)
    {}

    template class Properties<float>;
    template class Properties<double>;

  }
}
