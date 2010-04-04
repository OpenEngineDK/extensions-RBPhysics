

#ifndef   	TRIANGLEMESH_H_
# define   	TRIANGLEMESH_H_

#include <Geometry/FaceSet.h>
#include <Geometry/Geometry.h>
#include <Scene/ISceneNodeVisitor.h>
#include <Scene/ISceneNode.h>
#include <Scene/GeometryNode.h>
#include <string>
#include <vector>


namespace OpenEngine {
  namespace Geometry {


    using OpenEngine::Math::Vector;
    using namespace OpenEngine::Scene;
    using std::vector;

    class TriangleMesh : public GeometryBase {

    public:

      TriangleMesh() {}; // empty constructor for serialization

      explicit TriangleMesh(FaceSet& faces);
      explicit TriangleMesh(ISceneNode& node);


      FaceSet * GetFaceSet();

    private:

      FaceSet * faces;

      class FaceCollector : public ISceneNodeVisitor {
      private:
	FaceSet* faces;
      public:
	FaceCollector(ISceneNode& node) {
	  faces = new FaceSet();
	  node.Accept(*this);
	}
    
	virtual ~FaceCollector() {};
    
	void VisitGeometryNode(GeometryNode* node) {
	  faces->Add(node->GetFaceSet());
	}
     
	FaceSet* GetFaceSet() {
	  return faces;
	}
      };

    };
  }
}


#endif 	    /* !TRIANGLEMESH_H_ */
