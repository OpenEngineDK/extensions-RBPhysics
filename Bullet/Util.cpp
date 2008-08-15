#include <Bullet/Util.h>

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

namespace OpenEngine {
  namespace Bullet {


    OpenEngine::Math::Vector<3,float> toOEVec(const btVector3 & in) {
      return OpenEngine::Math::Vector<3,float>(in.x(),in.y(),in.z());
    }

    btVector3 toBtVec(const OpenEngine::Math::Vector<3,float> & in) {
      return btVector3(in.Get(0),in.Get(1),in.Get(2));
    }

    OpenEngine::Math::Quaternion<float> toOEQuat(const btQuaternion & in) {
      return OpenEngine::Math::Quaternion<float>(in.w(),in.x(),in.y(),in.z());
    }

    btQuaternion toBtQuat(const OpenEngine::Math::Quaternion<float> & in) {
      return btQuaternion(in.GetImaginary().Get(0),in.GetImaginary().Get(1),in.GetImaginary().Get(2),in.GetReal());
    }

  }
}
