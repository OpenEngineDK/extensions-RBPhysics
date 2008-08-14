#include "BulletRayResultCallback.h"

#include <Logging/Logger.h>
#include "Util.h"

using namespace OpenEngine::Logging;
using namespace OpenEngine::Physics;

namespace OpenEngine {
  namespace Bullet {

    BulletRayResultCallback::BulletRayResultCallback(IRayResultCallback * oeCallback, BulletEngine * engine) :
      RayResultCallback(),engine(engine),callback(oeCallback) {
    }
    
    btScalar BulletRayResultCallback::addSingleResult(btCollisionWorld::LocalRayResult & rayResult, bool normalInWorldSpace) {

//       logger.info << "hitNormal: " << toOEVec(rayResult.m_hitNormalLocal);
//       if(normalInWorldSpace) {
//         logger.info << " in world space" << logger.end;
//       }
//       else {
//         logger.info << " in local space" << logger.end;
//       }
//       logger.info << "Normal length: " << rayResult.m_hitNormalLocal.length() << logger.end;
//       logger.info << "HitFraction " << rayResult.m_hitFraction << logger.end;
//       logger.info << "closestFraction " << this->m_closestHitFraction << logger.end;
//       this->m_closestHitFraction = rayResult.m_hitFraction;
      IRigidBody * body = engine->LookUp(dynamic_cast<btRigidBody*>(rayResult.m_collisionObject));
      if(body) {
        callback->AddResult(IRayResultCallback::RayCastResult(body,
                                                              toOEVec(rayResult.m_hitNormalLocal),
                                                              normalInWorldSpace,
                                                              rayResult.m_hitFraction));
      }
      else {
        logger.error << "Could not find rigid body" << logger.end;
      }
      return 0;
    }

  }
}
