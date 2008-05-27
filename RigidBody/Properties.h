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

#ifndef _PROPERTIES_H
#define _PROPERTIES_H 1

#include <Math/Vector.h>
#include <Math/Matrix.h>

namespace OpenEngine {

  namespace RigidBody {

    /**
     * Constant and derived quantities for a Rigid Body
     *
     * @todo: should member access be restricted?
     * @todo: document members
     *
     * @class Properties Properties.h RigidBody/Properties.h
     */
    template <class Real>
      class Properties {

    public:
      /**
       * Create a new instance
       *
       * @param mass object mass
       * @param Ibody docthis
       * @param Ibodyinv docthis
       */
      Properties(const Real mass,
		 const Math::Matrix<3,3,Real> Ibody,
		 const Math::Matrix<3,3,Real> Ibodyinv);

      /* Constant quantities */
      const Real mass; //!< Mass of body
      const Math::Matrix<3,3,Real> Ibody; //!< docthis
      /*const*/ Math::Matrix<3,3,Real> Ibodyinv; //!< docthis

      /* Derived quantities */
      Math::Matrix<3,3,Real> Iinv; //!< docthis
      Math::Matrix<3,3,Real> R; //!< docthis
      Math::Vector<3,Real> v; //!< docthis
      Math::Vector<3,Real> omega; //!< docthis

      /* Computed quantities */
      Math::Vector<3,Real> force; //!< docthis
      Math::Vector<3,Real> torque; //!< docthis

    private:

    };
  }
}

#endif /* _PROPERTIES_H */

