#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"
#include <Renderers/IRenderer.h>
#include <Math/Vector.h>

class DebugDrawer : public btIDebugDraw
{
 private:
  int m_debugMode;
  OpenEngine::Renderers::IRenderer * renderer;
  OpenEngine::Math::Vector<3,float> p0;
  OpenEngine::Math::Vector<3,float> p1;
  OpenEngine::Math::Vector<3,float> p2;
  OpenEngine::Math::Vector<3,float> c;
 public:

  DebugDrawer(OpenEngine::Renderers::IRenderer * renderer);


  inline virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

  inline virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

  virtual	void	drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,
                                     const btVector3& n0,const btVector3& n1,const btVector3& n2,
                                     const btVector3& color, btScalar alpha);

  inline virtual void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar alpha);

  inline virtual void	reportErrorWarning(const char* warningString);

  inline virtual void	draw3dText(const btVector3& location,const char* textString);

  virtual void	setDebugMode(int debugMode);

  inline virtual int   getDebugMode() const { return m_debugMode;}

};

#endif//GL_DEBUG_DRAWER_H
