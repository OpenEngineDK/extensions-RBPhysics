#ifndef PHYSICS_CAR_CONFIG
#define PHYSICS_CAR_CONFIG
// 
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Math/Vector.h>

namespace OpenEngine {

  namespace Geometry 
    {
      class AABB;
    }
  
namespace Physics {

/**
 * A CarConfig is automatically generated for every Car. Changing most values
 * will only have an effect if done before adding the Car to the physics engine.
 *
 * @class CarConfig CarConfig.h ensions/RBPhysics/Physics/CarConfig.h
 */
class CarConfig {
public:
  CarConfig(const OpenEngine::Geometry::AABB & box);

  /**
   * Constant default values.
   */
  const int rightIndex;
  const int upIndex;
  const int forwardIndex;
  const OpenEngine::Math::Vector<3,float> wheelDirectionCS0;
  const OpenEngine::Math::Vector<3,float> wheelAxleCS;

  float suspensionRestLength; //!< Restlength of the suspension.
  
  float	wheelRadius; //!< Wheel radius. Calculated from the bounding box sorounding the vehicle.
  float	wheelWidth; //!< Wheel width. Calculated from the bounding box sorounding the vehicle.
  
  float	wheelFriction; //!< Wheel Friction. Default 5.
  float connectionHeight; //!< The height where the wheels are mounted to the chassis. A higher value will lower the car. Default 0.
  float	suspensionStiffness; //!< Stiffness of the suspension. Default 400.
  float	suspensionDamping; //!< Dampening of the suspension. A higher value will make the car less "jumpy". Default 40.
  float	suspensionCompression; //!< Suspension Compression. Default 0.
  float	rollInfluence; //!< Controls the influence of the ground on the rotation of the wheels. Default 1.3.

  float wheelDistanceWidth; //!< Half the length of the wheel axes. Default calculated from bounding box.
  float wheelDistanceLength; //!< Distance between the axes.

  float maxEngineForce; //!< Maximum force applied to the wheels by the engine. Default 100.
  float maxBreakForce; //!< Maximum force applied by the breaks. Default 10.
  
  /**
   * For car like vehicles (tankSteer == false) the maximum angle of the steering wheels.
   * For tank like vehicles (tankSteer == true) the maximum speed difference between the left and right wheels.
   * Default 0.3. For tanks it should be around 2.
   */
  float maxSteerAngle;

  /**
   * Should the vehicle apply the engine force to all wheels.
   * Default true.
   */
  bool allWheelDrive;
  bool allWheelSteer; //!< Should the vehicle steer with all wheels. Not the same as tank steer == true. Default false.

  /**
   * Setting this option to true makes the vehicle steer by applying different engine forces on
   * each side.
   * Default false.
   */
  bool tankSteer;

  /**
   * Gets the number of wheels on the vehicle.
   * 
   * @return The number of wheels on the vehicle.
   */
  int GetNumberOfWheels();
  /**
   * Set the number of wheels on the vehicle. The wheels are automatically
   * placed equally spaced.
   * 
   * @param The number of wheels on this vehicle.
   */
  void SetNumberOfWheels(int wheels);
  
  /**
   * Gets the breaking balance for this vehicle.
   *
   * @return The breaking balance.
   */
  float GetBreakingBalance();

  /**
   * Set the breaking balance on this vehicle. It should be in the range [0:1].
   * If set to 1 the vehicle will only break with the front wheels.
   * If set to 0 the vehicle will only break with the rear wheels.
   * 
   * @param balance The breaking balance.
   */
  void SetBreakingBalance(float balance);
  
 private:
  int numberOfWheels;

  float breakingBalance;
};

} // NS Physics
} // NS OpenEngine

#endif
