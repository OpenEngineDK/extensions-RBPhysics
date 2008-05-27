// Contact.
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

#ifndef _CONTACT_H
#define _CONTACT_H 1

#include "RigidBody.h"
#include <Math/Vector.h>

namespace OpenEngine {

  namespace RigidBody {

    /**
     * Contact point computed by collision detection
     *
     * @class Contact Contact.h RigidBody/Contact.h
     */
    template <class Real>
      class Contact {

    public:
      RigidBody<Real> * a; //!< body containing vertex
      RigidBody<Real> * b; //!< body containing face
      
      Math::Vector<3,Real> p; //!< world-space vertex location
      Math::Vector<3,Real> n; //!< outward pointing normal of face
      Math::Vector<3,Real> ea; //!< edge direction for A
      Math::Vector<3,Real> eb; //!< edge direction for B

      bool vf; //!< True if vertex/face contact

    private:

    };
  }
}

#endif /* _CONTACT_H */

