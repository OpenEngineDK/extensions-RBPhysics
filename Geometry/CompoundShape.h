#ifndef   	COMPOUNDSHAPE_H_
# define   	COMPOUNDSHAPE_H_

#include <Geometry/Geometry.h>
#include <Geometry/BoundingGeometry.h>
#include <Scene/TransformationNode.h>
#include <vector>



namespace OpenEngine {
  namespace Geometry {


    using OpenEngine::Math::Vector;
    using namespace OpenEngine::Scene;
    using std::vector;

    class CompoundShape : public Geometry {

    public:

      //CompoundShape() {}; // empty constructor for serialization

      explicit CompoundShape();

      void addChildShape(TransformationNode * trans, Geometry * childShape);

      int getNumChildShapes();

      Geometry * getChildShape(int index);

      TransformationNode * getChildTransform(int index);



    private:

      struct CompoundShapeChild {

	TransformationNode * trans;

	Geometry * childShape;
	
      };

      std::vector<CompoundShapeChild> children;


    };
  }
}


#endif 	    /* !COMPOUNDSHAPE_H_ */
