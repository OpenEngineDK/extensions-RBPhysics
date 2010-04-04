#ifndef _BULLETENGINE_H
#define _BULLETENGINE_H

#include <Physics/IPhysEngine.h>
#include <Math/Vector.h>
#include <list>
#include <Geometry/Geometry.h>
#include <boost/tuple/tuple.hpp>

class btRigidBody;
class btRaycastVehicle;
class btTransform;
class btCollisionShape;
class btDynamicsWorld;
class btBroadphaseInterface;
class btCollisionDispatcher;
class btConstraintSolver;
class btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


namespace OpenEngine 
{

  namespace Geometry 
    {
      class AABB;
    }

  namespace Physics 
    {
      class IRayResultCallback;
      class Car;
    }

  namespace Bullet
    {

      class BulletEngine : public Physics::IPhysEngine 
	{

	public:
          BulletEngine(OpenEngine::Geometry::AABB & worldAabb, OpenEngine::Math::Vector<3,float> gravity);
          ~BulletEngine();

          void Initialize();
  
          void Process(const float deltaTime);
  
          void Deinitialize();

	  void ClientResetScene(btRigidBody * chassis);
         
          void AddRigidBody(OpenEngine::Physics::IRigidBody * body);

          void RemoveRigidBody(OpenEngine::Physics::IRigidBody * body);

          OpenEngine::Physics::IRigidBody * LookUp(btRigidBody * body);
          btRigidBody * LookUp(OpenEngine::Physics::IRigidBody * body);

          void RayTest(const OpenEngine::Math::Vector<3,float> & begin,
                       const OpenEngine::Math::Vector<3,float> & end,
                       OpenEngine::Physics::IRayResultCallback * callback);

          OpenEngine::Scene::RenderNode *
	    getRenderNode(OpenEngine::Renderers::IRenderer * renderer);

          typedef boost::tuple<OpenEngine::Physics::IRigidBody*,btRigidBody*> BodyPair;
          typedef boost::tuple<OpenEngine::Physics::Car*,btRaycastVehicle*> CarPair;

        private:

          std::list< BodyPair > bodies;
          std::list< CarPair > cars;
  
          btRigidBody* localCreateRigidBody(float mass,
					    const btTransform& startTransform,
					    btCollisionShape* shape);

          /*
           * Helpers for AddRigidBody:
           */
	  btCollisionShape* ConvertShape(OpenEngine::Geometry::GeometryBase * geom);

          void CreateDynamicBody(OpenEngine::Physics::IRigidBody * body, 
                                 btCollisionShape * shape,
                                 btTransform trans);
 
          void CreateStaticBody(OpenEngine::Physics::IRigidBody * body, 
                                btCollisionShape * shape,
                                btTransform trans);


          ///this is the most important class
          btDynamicsWorld*    m_dynamicsWorld;
          btBroadphaseInterface* m_broadphase;  
          btCollisionDispatcher* m_dispatcher;
          btConstraintSolver* m_solver;
          btCollisionAlgorithmCreateFunc* m_boxBoxCF;
          btDefaultCollisionConfiguration* m_collisionConfiguration;  

          static const float collisionMargin = 0.05f;
          static const int maxProxies = 32766;
          static const int maxOverlap = 65535;  
          static const int maxNumObjects = 32760;
        };
    }
}


#endif
