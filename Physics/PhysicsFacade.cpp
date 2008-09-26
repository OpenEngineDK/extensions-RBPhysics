#include <Physics/PhysicsFacade.h>

#include <Bullet/BulletEngine.h>
#include <Physics/IRayResultCallback.h>
#include <Physics/IPhysEngine.h>
#include <Physics/IRigidBody.h>
#include <Renderers/IRenderNode.h>
#include <Renderers/IRenderer.h>
#include <Geometry/AABB.h>
#include <Math/Vector.h>
#include <Logging/Logger.h>

using namespace OpenEngine::Math;
using namespace OpenEngine::Core;
using namespace OpenEngine::Logging;

namespace OpenEngine {

  namespace Physics {

    PhysicsFacade::PhysicsFacade(OpenEngine::Geometry::AABB & worldAabb, Vector<3,float> gravity) 
    {
      physEngine =  new Bullet::BulletEngine(worldAabb,gravity);
      physEngine->Initialize();
    }


    void PhysicsFacade::Handle(InitializeEventArg arg) 
    {
    }
    
    void PhysicsFacade::Handle(ProcessEventArg arg)
    {
      float deltaTime = arg.approx / 1000.0;
      physEngine->Process(deltaTime);
    }
    
    void PhysicsFacade::Handle(DeinitializeEventArg arg) 
    {
      physEngine->Deinitialize();
    }

    void PhysicsFacade::AddRigidBody(IRigidBody * body) {
      physEngine->AddRigidBody(body);
    }


    void PhysicsFacade::RayTest(const Vector<3,float> & begin, const Vector<3,float> & end, IRayResultCallback * callback) {
      physEngine->RayTest(begin,end,callback);
    }
//     void PhysicsFacade::AddStaticGeometry(StaticGeometry * geometry) {
//       physEngine->AddStaticGeometry(geometry);
//     }



    void PhysicsFacade::RemoveRigidBody(IRigidBody * body) {
      physEngine->RemoveRigidBody(body);
    }

    OpenEngine::Renderers::IRenderNode * PhysicsFacade::getRenderNode(OpenEngine::Renderers::IRenderer * renderer) {
      return physEngine->getRenderNode(renderer);
    }
  }
}
