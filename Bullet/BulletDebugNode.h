#ifndef PHYSICS_BULLET_DEBUG_NODE
#define PHYSICS_BULLET_DEBUG_NODE

#include <Scene/RenderNode.h>
#include <Scene/ISceneNodeVisitor.h>
#include <btBulletDynamicsCommon.h>
#include <iostream>

namespace OpenEngine 
{
  namespace Bullet 
    {

class BulletDebugNode : public OpenEngine::Scene::RenderNode
{

 public:

  BulletDebugNode(btDynamicsWorld* dynamicsWorld) :
    m_dynamicsWorld(dynamicsWorld)
    {}

  btDynamicsWorld* m_dynamicsWorld;

  void Apply(Renderers::RenderingEventArg arg, OpenEngine::Scene::ISceneNodeVisitor& v) 
  {
        m_dynamicsWorld->debugDrawWorld();
    //    btVector3 pos = m_ballBody->getCenterOfMassPosition();
    //    cout << "(" << pos[0] << "," << pos[1] << "," << pos[2] << ")" <<  endl;
/*     m_camera->SetPosition(Vector<3,float>(pos[0],pos[1],pos[2]+50)); */
/*     m_camera->LookAt(pos[0],pos[1],pos[2]); */

/*     m_camera->SetPosition(Vector<3,float>(pos[0],pos[1]+5,pos[2])); */
/*     m_camera->LookAt(0,5,0); */
  }
};
    }
}


#endif
