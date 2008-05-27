
#include "DebugDrawer.h"

#include "LinearMath/btPoint3.h"

#include "Util.h"
#include <Geometry/Line.h>
#include <Logging/Logger.h>
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Logging;
using namespace OpenEngine::Bullet;

DebugDrawer::DebugDrawer(OpenEngine::Renderers::IRenderer * renderer)
  :m_debugMode(1),
   renderer(renderer)
{

}
void	DebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
    renderer->DrawLine(Line(toOEVec(from),toOEVec(to)),toOEVec(color));
}

void	DebugDrawer::setDebugMode(int debugMode)
{
  m_debugMode = debugMode;
}

void	DebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
// 	glRasterPos3f(location.x(),  location.y(),  location.z());
// 	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void	DebugDrawer::reportErrorWarning(const char* warningString)
{
  logger.error << warningString << logger.end;
}

void	DebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
  const Vector<3,float> to = toOEVec(pointOnB+normalOnB*distance);
  const Vector<3,float> from = toOEVec(pointOnB);
  renderer->DrawLine(Line(from,to),toOEVec(color));
// 	if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
// 	{
// 		btVector3 to=pointOnB+normalOnB*distance;
// 		const btVector3&from = pointOnB;
// 		glBegin(GL_LINES);
// 		glColor3f(color.getX(), color.getY(), color.getZ());
// 		glVertex3d(from.getX(), from.getY(), from.getZ());
// 		glVertex3d(to.getX(), to.getY(), to.getZ());
// 		glEnd();


// 		glRasterPos3f(from.x(),  from.y(),  from.z());
// 		char buf[12];
// 		sprintf(buf," %d",lifeTime);
//                 //		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


// 	}
}




