#ifndef   	HEIGHTFIELDTERRAINSHAPE_H_
# define   	HEIGHTFIELDTERRAINSHAPE_H_

#include <Geometry/Geometry.h>
#include <Resources/Texture2D.h>
#include <string>
#include <vector>
#include <Math/Vector.h>

namespace OpenEngine {
  namespace Geometry {


    using OpenEngine::Math::Vector;
    using std::vector;
      using namespace Resources;

      /**
       * HeightFieldTerrainShape
       * Physics collision shape that represents a heightmap given by
       * a ITextureResource
       *
       * @param tex Texture from which the heightmap is loaded
       * @param maxHeight maximum heigth
       * @param scaling scaling on the other axis
       * @param upAxis number of the axis pointing upwards 0=x 1=y 2=z
       * @param useFloatData true if input is in floats should be
       * false for TGAResources
       * @param flipQuadEdges false= |/|/|/|/| true=|/|\|/|\| probably?!?
       *
       */

    class HeightfieldTerrainShape : public GeometryBase {

    public:

      HeightfieldTerrainShape() {}; // empty constructor for serialization

        explicit HeightfieldTerrainShape(FloatTexture2DPtr tex, 
                                         float maxHeight, 
                                         float scaling,
                                         int upAxis,
                                         bool useFloatData, 
                                         bool flipQuadEdges);

      float GetHeightFieldValue(int x, int y);

      FloatTexture2DPtr GetTextureResource();

      float GetMaxHeight();

      float GetScaling();

      int GetUpAxis();

      bool UsesFloatData();

      bool FlippedQuadEdges();


    private:

  //      OpenEngine::Resources::ITextureResource * tex;
      FloatTexture2DPtr tex;
      float maxHeight, scaling;
      int upAxis;
      bool useFloatData, flipQuadEdges;

    };
  }
}



#endif 	    /* !HEIGHTFIELDTERRAINSHAPE_H_ */
