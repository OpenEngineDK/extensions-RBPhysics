#ifndef _BULLET_UTIL_H
#define _BULLET_UTIL_H

#include <Math/Vector.h>
#include <Math/Quaternion.h>

class btVector3;
class btQuaternion;

namespace OpenEngine {
  namespace Bullet {

    OpenEngine::Math::Vector<3,float> toOEVec(const btVector3 & in);

    btVector3 toBtVec(const OpenEngine::Math::Vector<3,float> & in);

    OpenEngine::Math::Quaternion<float> toOEQuat(const btQuaternion & in);

    btQuaternion toBtQuat(const OpenEngine::Math::Quaternion<float> & in);

  }
}
#endif /* _UTIL_H */

