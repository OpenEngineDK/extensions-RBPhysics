#include "TriangleMesh.h"

#include <Geometry/Face.h>
#include <Geometry/Line.h>
#include <Geometry/Plane.h>
#include <Core/Exceptions.h>

namespace OpenEngine {
namespace Geometry {

    using OpenEngine::Math::Vector;

    TriangleMesh::TriangleMesh(ISceneNode* node)
	{
    	GeometryCollector gc(node);
    	meshs = gc.GetMeshes();
	}

	TriangleMesh::~TriangleMesh()
	{
		delete meshs;
	}
}}





