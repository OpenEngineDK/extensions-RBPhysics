
#include "HeightfieldTerrainShape.h"

namespace OpenEngine {
  namespace Geometry {



    HeightfieldTerrainShape::HeightfieldTerrainShape(IDataBlockPtr block,
                                                     int width,
                                                     int depth,
                                                     float maxHeight,
                                                     float scaling,
                                                     int upAxis,
                                                     bool useFloatData, 
                                                     bool flipQuadEdges) 
        : block(block), 
          width(width),
          depth(depth),
          maxHeight(maxHeight), scaling(scaling), upAxis(upAxis),
	useFloatData(useFloatData), flipQuadEdges(flipQuadEdges)
    {
      
    }


    float HeightfieldTerrainShape::GetHeightFieldValue(int x, int y)
    {
      // is this correct?
        return 0;
        //return tex->GetData()[x * tex->GetWidth() + y] * maxHeight;
    }


    // FloatTexture2DPtr HeightfieldTerrainShape::GetTextureResource()
    // {
    //   return tex;
    // }


      int HeightfieldTerrainShape::GetWidth() { return width; }
      int HeightfieldTerrainShape::GetDepth() { return depth; }

      IDataBlockPtr HeightfieldTerrainShape::GetDataBlock() { return block; }

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
