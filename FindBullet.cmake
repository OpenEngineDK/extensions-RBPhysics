# - Try to find Bullet
# Once done this will define
#
#  BULLET_FOUND - system has bullet
#  BULLET_INCLUDE_DIR - the bullet include directory
#  BULLET_LIBRARIES - Link these to use Collada DOM
#

STRING(COMPARE EQUAL ${CMAKE_BUILD_TYPE} "debug" ISDEBUGENABLED)

# IF(ISDEBUGENABLED)
#   SET(BULLETDYNAMICS "bulletdynamics-d")
#   SET(BULLETCOLLISION "bulletcollision-d")
#   SET(BULLETMATH "bulletmath-d")
#   SET(BULLETSOFTBODY "bulletsoftbody-d")  
# ELSE(ISDEBUGENABLED)
  SET(BULLETDYNAMICS "bulletdynamics")
  SET(BULLETCOLLISION "bulletcollision")
  SET(BULLETMATH "linearmath")
  SET(BULLETSOFTBODY "bulletsoftbody")  
# ENDIF(ISDEBUGENABLED)

FIND_PATH(BULLET_INCLUDE_DIR NAMES btBulletCollisionCommon.h btBulletCollisionCommon.h
  PATHS
  ${PROJECT_BINARY_DIR}/include
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/libraries/bullet-2.76/src
  ENV CPATH
  /usr/include
  /usr/local/include
  /opt/local/include
  NO_DEFAULT_PATH
)

MESSAGE("${BULLET_INCLUDE_DIR}")

FIND_LIBRARY(LIBBULLETDYNAMICS
  NAMES 
  ${BULLETDYNAMICS}
  PATHS
  ${PROJECT_BINARY_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/libraries
  ${PROJECT_SOURCE_DIR}/libraries/bullet-2.76/src/BulletDynamics
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

MESSAGE("${LIBBULLETDYNAMICS}")


IF(NOT LIBBULLETDYNAMICS)
  MESSAGE ("WARNING: Could not find bullet dynamics - depending targets will be disabled.")
ENDIF(NOT LIBBULLETDYNAMICS)


FIND_LIBRARY(LIBBULLETCOLLISION
  NAMES 
  ${BULLETCOLLISION}
  PATHS
  ${PROJECT_BINARY_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/libraries
  ${PROJECT_SOURCE_DIR}/libraries/bullet-2.76/src/BulletCollision
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

IF(NOT LIBBULLETCOLLISION)
  MESSAGE ("WARNING: Could not find bullet collision - depending targets will be disabled.")
ENDIF(NOT LIBBULLETCOLLISION)

FIND_LIBRARY(LIBBULLETMATH
  NAMES 
  ${BULLETMATH}
  PATHS
  ${PROJECT_BINARY_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/libraries
  ${PROJECT_SOURCE_DIR}/libraries/bullet-2.76/src/LinearMath
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

IF(NOT LIBBULLETMATH)
  MESSAGE ("WARNING: Could not find bullet math - depending targets will be disabled.")
ENDIF(NOT LIBBULLETMATH)

FIND_LIBRARY(LIBBULLETSOFTBODY
  NAMES 
  ${BULLETSOFTBODY}
  PATHS
  ${PROJECT_BINARY_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/libraries
  ${PROJECT_SOURCE_DIR}/libraries/bullet-2.76/src/BulletSoftBody
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

IF(NOT LIBBULLETSOFTBODY)
  MESSAGE ("WARNING: Could not find bulletsoftbody - depending targets will be disabled.")
ENDIF(NOT LIBBULLETSOFTBODY)


SET(BULLET_LIBRARIES ${LIBBULLETDYNAMICS} ${LIBBULLETCOLLISION} ${LIBBULLETMATH} ${LIBBULLETSOFTBODY})

IF(BULLET_INCLUDE_DIR AND BULLET_LIBRARIES)
  SET(BULLET_FOUND TRUE)
ENDIF(BULLET_INCLUDE_DIR AND BULLET_LIBRARIES)

# show the BULLET_INCLUDE_DIR and BULLET_LIBRARIES variables only in the advanced view
IF(BULLET_FOUND)
  MARK_AS_ADVANCED(BULLET_INCLUDE_DIR BULLET_DOM_LIBRARIES )
ENDIF(BULLET_FOUND)


