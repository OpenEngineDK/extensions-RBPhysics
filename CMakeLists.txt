
# Create the extension library
ADD_LIBRARY( Extensions_RBPhysics
  RigidBody/Properties.cpp
  RigidBody/RigidBody.cpp
  Physics/PhysicsFacade.cpp
  Physics/RigidBody.cpp
  Physics/DynamicBody.cpp
  Physics/RigidBodyDecorator.cpp
  Physics/RayPrintNameCallback.cpp
  Physics/Car.cpp
  Bullet/BulletEngine.cpp
  Bullet/DebugDrawer.cpp
  Bullet/Util.cpp
  Bullet/BulletRayResultCallback.cpp
  Geometry/TriangleMesh.cpp
  Geometry/CompoundShape.cpp
  Geometry/HeightfieldTerrainShape.cpp
  Geometry/AABB.cpp
)

TARGET_LINK_LIBRARIES( Extensions_RBPhysics
  ${BULLET_LIBRARIES}
)
