
#include "HeightfieldTerrainShape.h"

namespace OpenEngine {
  namespace Geometry {



    HeightfieldTerrainShape::HeightfieldTerrainShape(FloatTexture2DPtr tex,
						     float maxHeight, float scaling, int upAxis,
						     bool useFloatData, bool flipQuadEdges) 
      : tex(tex), maxHeight(maxHeight), scaling(scaling), upAxis(upAxis),
	useFloatData(useFloatData), flipQuadEdges(flipQuadEdges)
    {
      tex->Load();
    }


    float HeightfieldTerrainShape::GetHeightFieldValue(int x, int y)
    {
      // is this correct?
      return tex->GetData()[x * tex->GetWidth() + y] * maxHeight;
    }


    FloatTexture2DPtr HeightfieldTerrainShape::GetTextureResource()
    {
      return tex;
    }


    float HeightfieldTerrainShape::GetMaxHeight()
    {
      return maxHeight;
    }

    float HeightfieldTerrainShape::GetScaling()
    {
      return scaling;
    }

    int HeightfieldTerrainShape::GetUpAxis()
    {
      return upAxis;
    }

    bool HeightfieldTerrainShape::UsesFloatData()
    {
      return useFloatData;
    }

    bool HeightfieldTerrainShape::FlippedQuadEdges()
    {
      return flipQuadEdges;
    }


  }
}
