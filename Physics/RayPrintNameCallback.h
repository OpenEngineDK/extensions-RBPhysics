/* @(#)RayPrintNameCallback.h
 */

#ifndef _RAYPRINTNAMECALLBACK_H
#define _RAYPRINTNAMECALLBACK_H 1

#include <Physics/IRayResultCallback.h>

namespace OpenEngine {
  namespace Physics {

    /**
     * This callback will print the name of a rigid body
     * when it is hit by the ray.
     * This class is intended for debugging.
     */
    class RayPrintNameCallback : public IRayResultCallback {
      
      void AddResult(RayCastResult result);

    };

  }
}

#endif /* _RAYPRINTNAMECALLBACK_H */

