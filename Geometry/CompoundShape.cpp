#include "CompoundShape.h"


namespace OpenEngine {
  namespace Geometry {

    using OpenEngine::Scene::TransformationNode;

    CompoundShape::CompoundShape() {
      children = vector<CompoundShapeChild>();
    }

    void CompoundShape::addChildShape(TransformationNode * trans, BoundingGeometry * childShape) {
      CompoundShapeChild child;
      child.trans = trans;
      child.childShape = childShape;
      children.push_back(child);
    }


      int CompoundShape::getNumChildShapes()
      {
	return children.size();
      }

      BoundingGeometry * CompoundShape::getChildShape(int index)
      {
	return children[index].childShape;
      }

      TransformationNode * CompoundShape::getChildTransform(int index)
      {
	return children[index].trans;
      }



  }
}
