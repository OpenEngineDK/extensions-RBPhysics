#ifndef   	TRIANGLEMESH_H_
#define   	TRIANGLEMESH_H_

#include <Geometry/FaceSet.h>
#include <Geometry/Geometry.h>
#include <Geometry/GeometrySet.h>
#include <Geometry/Mesh.h>
#include <Scene/ISceneNodeVisitor.h>
#include <Scene/ISceneNode.h>
#include <Scene/GeometryNode.h>
#include <Scene/MeshNode.h>
#include <string>
#include <vector>
#include <iostream>
using namespace std;

namespace OpenEngine {
namespace Geometry {

using OpenEngine::Math::Vector;
using namespace OpenEngine::Scene;
using std::vector;

class TriangleMesh : public GeometryBase
{
public:
	TriangleMesh(){}; // empty constructor for serialization

    explicit TriangleMesh(ISceneNode* node);

	Resources::IDataBlockPtr GetVertices()
	{
		return Vertices;
	}

	IndicesPtr GetIndices()
	{
		return Indices;
	}

private:
	Resources::IDataBlockPtr Vertices;
	IndicesPtr Indices;

    class GeometryCollector : public ISceneNodeVisitor
	{
    private:
		Resources::IDataBlockPtr Vertices;
		IndicesPtr Indices;
 
	public:
		GeometryCollector(ISceneNode* node)
		{
	  		node->Accept(*this);
		}
    
		virtual ~GeometryCollector() {};
    
		void VisitMeshNode(MeshNode* node)
		{
			//Tjek form, vha getType på meshNode
			//Add support for more than one meshNode	
			
			Vertices = node->GetMesh()->GetGeometrySet()->GetVertices();
			Indices = node->GetMesh()->GetIndices();
		}
     
		Resources::IDataBlockPtr GetVertices()
		{
			return Vertices;
		}

		IndicesPtr GetIndices()
		{
			return Indices;
		}
	}; // FaceCollector
}; //TriangleMesh

}}


#endif 	    /* !TRIANGLEMESH_H_ */
