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

	~TriangleMesh();

	MeshNode* GetMesh(int i)
	{
		return (*meshs)[i];
	}

	unsigned int GetSize()
	{
		return meshs->size();
	}
private:
	std::vector<MeshNode*> *meshs;

    class GeometryCollector : public ISceneNodeVisitor
	{
    private:
		std::vector<MeshNode*> *meshs;
	public:
		GeometryCollector(ISceneNode* node)
		{
			meshs=new std::vector<MeshNode*>();
	  		node->Accept(*this);
		}
    
		virtual ~GeometryCollector() {};
    
		void VisitMeshNode(MeshNode* node)
		{
			meshs->push_back(node);	
		}

		std::vector<MeshNode*>* GetMeshes()
		{
			return meshs;
		}
	}; // FaceCollector
}; //TriangleMesh

}}

#endif 	    /* !TRIANGLEMESH_H_ */
