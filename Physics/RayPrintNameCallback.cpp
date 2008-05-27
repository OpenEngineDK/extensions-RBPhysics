#include "RayPrintNameCallback.h"

#include <Logging/Logger.h>

using namespace OpenEngine::Logging;

namespace OpenEngine {
  namespace Physics {

    void RayPrintNameCallback::AddResult(RayCastResult result) {
      logger.info << result.bodyHit->GetName() << logger.end;
    }

  }
}
