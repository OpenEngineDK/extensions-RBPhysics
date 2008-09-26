#ifndef _IPHYSENGINE_H
#define _IPHYSENGINE_H

#include <Renderers/IRenderNode.h>

namespace OpenEngine 
{
  namespace Physics
  {

    class IRigidBody;
    class IRayResultCallback;

    class IPhysEngine 
    {
    public:

      virtual ~IPhysEngine() {};

      // Methods as in IModule
      virtual void Initialize() = 0;
      virtual void Process(const float deltaTime) = 0;
      virtual void Deinitialize() = 0;


      /**
       * Add a rigid body to the physics engine.
       * The parameters of the rigid body will be updated automatically after the
       * engine is started.
       *
       * @param body A pointer to a rigid body that will be simulated by the engine.
       */
      virtual void AddRigidBody(OpenEngine::Physics::IRigidBody * body) = 0;

      virtual void RemoveRigidBody(OpenEngine::Physics::IRigidBody * body) = 0;

      /**
       * Test the ray given by begin and end for intersection.
       * The call back object will be called if the ray hit a rigid body.
       *
       * @param begin The start of the ray.
       * @param end The end of the ray.
       * @param callback The callback that will be invoked on a hit.
       */
      virtual void RayTest(const OpenEngine::Math::Vector<3,float> & begin, 
                           const OpenEngine::Math::Vector<3,float> & end,
                           IRayResultCallback * callback) = 0;

      /**
       * Get the render node for debugging.
       *
       * @param renderer The renderer that should be used to render the debug information
       * @return A Render node that should be added to the root of the scene.
       */
      virtual OpenEngine::Renderers::IRenderNode * getRenderNode(OpenEngine::Renderers::IRenderer * renderer) = 0;
    };
  }
}


#endif
