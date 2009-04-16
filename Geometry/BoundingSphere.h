#ifndef _OE_BOUNDING_SPHERE_
#define _OE_BOUNDING_SPHERE_

// dummy class, to make RBPhysics compile again

namespace OpenEngine {
namespace Geometry {

class BoundingSphere : public Geometry {
    private:
        Sphere* sphere;
    public:
        BoundingSphere(Vector<3,float> center, float diameter) {
            sphere = new Sphere(center,diameter);
        }
        virtual ~BoundingSphere() {}
};

} // NS Geometry
} // NS OpenEngine

#endif // _OE_BOUNDING_SPHERE_
