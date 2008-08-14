#include <Physics/PhysicsFacade.h>
#include <Bullet/BulletEngine.h>

using namespace OpenEngine::Math;

namespace OpenEngine {

  namespace Physics {

    PhysicsFacade::PhysicsFacade(OpenEngine::Geometry::AABB & worldAabb, Vector<3,float> gravity) 
    {
      physEngine =  new Bullet::BulletEngine(worldAabb,gravity);
      physEngine->Initialize();
    }

    void PhysicsFacade::Initialize() 
    {

    }


    void PhysicsFacade::Process(const float deltaTime, const float percent) 
    {
      physEngine->Process(deltaTime, percent);
    }

    void PhysicsFacade::Deinitialize() 
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


    bool PhysicsFacade::IsTypeOf(const std::type_info& inf) 
    {
      return typeid(PhysicsFacade) == inf;
    }

    OpenEngine::Renderers::IRenderNode * PhysicsFacade::getRenderNode(OpenEngine::Renderers::IRenderer * renderer) {
      return physEngine->getRenderNode(renderer);
    }
  }
}
