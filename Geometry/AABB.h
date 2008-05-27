// Box.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _AABOUNDING_BOX_H_
#define _AABOUNDING_BOX_H_

#include <Geometry/FaceSet.h>
#include <Geometry/BoundingGeometry.h>
#include <Scene/ISceneNode.h>
#include <Scene/GeometryNode.h>
#include <string>
#include <vector>

namespace OpenEngine {
namespace Geometry {

using OpenEngine::Math::Vector;
using namespace OpenEngine::Scene;
using std::vector;


/**
 * Bounding geometry box.
 *
 * @class Box Box.h Geometry/Box.h
 */
class AABB : public BoundingGeometry {

friend class Geometry;
    
private:
    
    Vector<3,float> center;     //!< Box center
    Vector<3,float> corner;     //!< Box corner (relative)

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & center;
        ar & corner;
    }


public:
    AABB() {}; // empty constructor for serialization

    AABB(const Vector<3,float> & center, const Vector<3,float> & corner);

    explicit AABB(FaceSet& faces);
    explicit AABB(ISceneNode& node);

    void SetCenter(Vector<3,float> center);
    Vector<3,float> GetCenter() const;
    Vector<3,float> GetCorner() const;
    Vector<3,float> GetCorner(const int index) const;
    Vector<3,float> GetCorner(const bool signX, const bool signY, const bool signZ) const;
    //    void SetCorner(const bool x, const bool y, const bool z, Vector<3,float> c);
    void SetCorner(Vector<3,float> c);
    bool Intersects(const Vector<3,float> point) const;
    bool Intersects(const Line line) const;
    bool Intersects(const Plane plane) const;

};

} //NS Common
} //NS OpenEngine

#endif
