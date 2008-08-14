/* @(#)BulletRayResultCallback.h
 */

#ifndef _BULLETRAYRESULTCALLBACK_H
#define _BULLETRAYRESULTCALLBACK_H 1

#include "BulletEngine.h"
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace OpenEngine {
  namespace Bullet {

    class BulletRayResultCallback : public btCollisionWorld::RayResultCallback {
      
    public:
      BulletRayResultCallback(OpenEngine::Physics::IRayResultCallback * oeCallback,
                              BulletEngine * engine);

      btScalar addSingleResult(btCollisionWorld::LocalRayResult & rayResult, bool normalInWorldSpace);

    private:
      BulletEngine * engine;
      OpenEngine::Physics::IRayResultCallback * callback;
    };
  }
}
#endif /* _BULLETRAYRESULTCALLBACK_H */

