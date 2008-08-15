// 
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Physics/CarConfig.h>
#include <Logging/Logger.h>
#include <Geometry/AABB.h>

using namespace OpenEngine::Logging;
using namespace OpenEngine::Math;


namespace OpenEngine {
namespace Physics {

  CarConfig::CarConfig(const OpenEngine::Geometry::AABB & box) :
    rightIndex(0),
    upIndex(1),
    forwardIndex(2),
    wheelDirectionCS0(0,-1,0),
    wheelAxleCS(0,0,1),
    suspensionRestLength(0.0),
  
    wheelRadius(box.GetCorner().GetLength()/2),
    wheelWidth(box.GetCorner().GetLength()/5),
  
    wheelFriction(5),
    connectionHeight(0.f),
    suspensionStiffness(400.f),
    suspensionDamping(40.0f),
    suspensionCompression(0.0f),
    rollInfluence(1.3),

    wheelDistanceWidth(box.GetCorner().Get(2)),
    wheelDistanceLength(box.GetCorner().Get(0)),
  
    maxEngineForce(100),
    maxBreakForce(10),
    maxSteerAngle(0.3),

    allWheelDrive(true),
    allWheelSteer(false),

    tankSteer(false),
  
    numberOfWheels(4),
    
    breakingBalance(0.5)

  {}

  int CarConfig::GetNumberOfWheels() 
  {
    return numberOfWheels;
  }
  
  void CarConfig::SetNumberOfWheels(int wheels)
  {
    if(wheels%2 != 0) {
      logger.warning << "number of wheels should be dividable by 2" << logger.end;
    }
    this->numberOfWheels = wheels;
  }
  
  
  float CarConfig::GetBreakingBalance() 
  {
    return breakingBalance;
  }
  
  void CarConfig::SetBreakingBalance(float balance) 
  {
    if(breakingBalance > 1 || breakingBalance < 0) {
      logger.warning << "breaking balance should be in the range [0:1]" << logger.end;
    }
    this->breakingBalance = balance;
  }
  

} // NS Physics
} // NS OpenEngine
