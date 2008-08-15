#include <Bullet/DebugDrawer.h>

#include <LinearMath/btPoint3.h>
#include <Bullet/Util.h>
#include <Geometry/Line.h>
#include <Geometry/Face.h>
#include <Logging/Logger.h>
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Logging;
using namespace OpenEngine::Bullet;

DebugDrawer::DebugDrawer(OpenEngine::Renderers::IRenderer * renderer)
  :m_debugMode(1),
   renderer(renderer)
{

}
inline void	DebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
    renderer->DrawLine(Line(toOEVec(from),toOEVec(to)),toOEVec(color));
}

void DebugDrawer::drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,
                          const btVector3& n0,const btVector3& n1,const btVector3& n2,
                          const btVector3& color, btScalar alpha) 
{
  p0 = toOEVec(v0);
  p1 = toOEVec(v1);
  p2 = toOEVec(v2);
  c = toOEVec(color);
  renderer->DrawLine(Line(p0,p1),c);
  renderer->DrawLine(Line(p1,p2),c);
  renderer->DrawLine(Line(p2,p0),c);
}

inline void DebugDrawer::drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar alpha) 
{
  p0 = toOEVec(v0);
  p1 = toOEVec(v1);
  p2 = toOEVec(v2);
  c = toOEVec(color);
  renderer->DrawLine(Line(p0,p1),c);
  renderer->DrawLine(Line(p1,p2),c);
  renderer->DrawLine(Line(p2,p0),c);
}


inline void	DebugDrawer::setDebugMode(int debugMode)
{
  m_debugMode = debugMode;
}

inline void	DebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
}

inline void	DebugDrawer::reportErrorWarning(const char* warningString)
{
  logger.error << warningString << logger.end;
}

inline void	DebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
  p1 = toOEVec(pointOnB+normalOnB*distance);
  p0 = toOEVec(pointOnB);
  c = toOEVec(color);
  renderer->DrawLine(Line(p0,p1),c);
}




