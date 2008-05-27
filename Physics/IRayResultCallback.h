/* @(#)IRayResultCallback.h
 */

#ifndef _IRAYRESULTCALLBACK_H
#define _IRAYRESULTCALLBACK_H 1

#include "IRigidBody.h"
#include <Math/Vector.h>

namespace OpenEngine {
  namespace Physics {
    class IRayResultCallback {

    public:

      class RayCastResult {
        
      public:
      RayCastResult(IRigidBody * body,
                    OpenEngine::Math::Vector<3,float> normal,
                    bool normalInWorldSpace,
                    float fraction) : 
        bodyHit(body),
          normal(normal),
          normalInWorldSpace(normalInWorldSpace),
          fraction(fraction)
        {}

        IRigidBody * bodyHit;
        OpenEngine::Math::Vector<3,float> normal;
        bool normalInWorldSpace;
        float fraction;
      };

      virtual void AddResult(RayCastResult result) = 0;
    };
  }
}

#endif /* _IRAYRESULTCALLBACK_H */

