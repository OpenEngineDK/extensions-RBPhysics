#include <Renderers/IRenderNode.h>
#include <Renderers/IRenderingView.h>
#include <src/btBulletDynamicsCommon.h>
#include <iostream>

class BulletDebugNode : public OpenEngine::Renderers::IRenderNode
{

 public:

  BulletDebugNode(btDynamicsWorld* dynamicsWorld) :
    m_dynamicsWorld(dynamicsWorld)
    {}

  btDynamicsWorld* m_dynamicsWorld;

  void Apply(OpenEngine::Renderers::IRenderingView* view) 
  {
    using namespace std;


    m_dynamicsWorld->debugDrawWorld();
    //    btVector3 pos = m_ballBody->getCenterOfMassPosition();
    //    cout << "(" << pos[0] << "," << pos[1] << "," << pos[2] << ")" <<  endl;
/*     m_camera->SetPosition(Vector<3,float>(pos[0],pos[1],pos[2]+50)); */
/*     m_camera->LookAt(pos[0],pos[1],pos[2]); */

/*     m_camera->SetPosition(Vector<3,float>(pos[0],pos[1]+5,pos[2])); */
/*     m_camera->LookAt(0,5,0); */
  }
};
