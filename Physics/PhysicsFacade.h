#ifndef _PHYSICSFACADE_H
#define _PHYSICSFACADE_H 1

#include <Core/IModule.h>
#include <Math/Vector.h>

namespace OpenEngine {

  namespace Renderers 
    {
      class IRenderNode;
      class IRenderer;
    }

  namespace Geometry 
    {
      class AABB;
    }

  namespace Physics {

    class IRayResultCallback;
    class IRigidBody;
    class IPhysEngine;
    
    /**
     * This is the main interface to the physics engine.
     * It will create a instance of a concrete physics engine
     * and pass all calls to it.
     */
    class PhysicsFacade : public Core::IModule {
    public:

      PhysicsFacade(OpenEngine::Geometry::AABB & worldAabb, OpenEngine::Math::Vector<3,float> gravity);

      // Methods inherited from IModule

      void Handle(OpenEngine::Core::InitializeEventArg arg);
      void Handle(OpenEngine::Core::ProcessEventArg arg);
      void Handle(OpenEngine::Core::DeinitializeEventArg arg);

      /**
       * Get the render node for debugging.
       *
       * @param renderer The renderer that should be used to render the debug information
       * @return A Render node that should be added to the root of the scene.
       */
      OpenEngine::Renderers::IRenderNode * getRenderNode(OpenEngine::Renderers::IRenderer * renderer);

      /**
       * Add a rigid body to the physics engine.
       * The parameters of the rigid body will be updated automatically after the
       * engine is started.
       *
       * @param body A pointer to a rigid body that will be simulated by the engine.
       */
      void AddRigidBody(IRigidBody * body);
      void RemoveRigidBody(IRigidBody * body);

      /**
       * Test the ray given by begin and end for intersection.
       * The call back object will be called if the ray hit a rigid body.
       *
       * @param begin The start of the ray.
       * @param end The end of the ray.
       * @param callback The callback that will be invoked on a hit.
       */
      void RayTest(const OpenEngine::Math::Vector<3,float> & begin, 
                   const OpenEngine::Math::Vector<3,float> & end, 
                   IRayResultCallback * callback);

      //      void AddStaticGeometry(StaticGeometry * geometry);

    private:
      IPhysEngine * physEngine;
    };
  }
}

#endif /* _PHYSICSFACADE_H */
