/* @(#)BulletRayResultCallback.h
 */

#ifndef _BULLETRAYRESULTCALLBACK_H
#define _BULLETRAYRESULTCALLBACK_H 1

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace OpenEngine {

  namespace Physics 
    {
      class IRayResultCallback;
    }

  namespace Bullet {

    class BulletEngine;

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

