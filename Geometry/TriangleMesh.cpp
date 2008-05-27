#include "TriangleMesh.h"

#include <Geometry/Face.h>
#include <Geometry/Line.h>
#include <Geometry/Plane.h>
#include <Core/Exceptions.h>


namespace OpenEngine {
  namespace Geometry {

    using OpenEngine::Math::Vector;

    TriangleMesh::TriangleMesh(FaceSet& faces) : faces(&faces) {

    }


    TriangleMesh::TriangleMesh(ISceneNode& node) {
    
      FaceCollector fc(node);
      faces = fc.GetFaceSet();

    }

    FaceSet * TriangleMesh::GetFaceSet() {
      return faces;
    }
  }
}





