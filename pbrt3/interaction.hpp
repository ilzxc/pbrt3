//
//  interaction.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/12/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef interaction_hpp
#define interaction_hpp

#include "geometry.hpp"
#include "pbrt.hpp"

namespace pbrt {

class MediumInterface;

struct Interaction
{
    Point3f p;
    Float time;
    Vector3f pError;
    Vector3f wo;
    Normal3f n;
    MediumInterface* mediumInterface; // change to not-a-pointer after Chapter 11.3.1

    Interaction() {}

    Interaction( const Point3f& p, const Normal3f& n, const Vector3f& pError, const Vector3f& wo,
                 Float time, MediumInterface* mediumInterface );

    bool IsSurfaceInteraction() const { return n != Normal3f(); }
};

struct SurfaceInteraction : public Interaction
{
    Point2f uv;
    Vector3f dpdu, dpdv;
    Normal3f dndu, dndv;
    const Shape* shape = nullptr;

    struct
    {
        Normal3f n;
        Vector3f dpdu, dpdv;
        Normal3f dndu, dndv;
    } shading;

    SurfaceInteraction() : Interaction{} {}

    SurfaceInteraction( const Point3f& p, const Vector3f& pError, const Point2f& uv,
                        const Vector3f& wo, const Vector3f& dpdu, const Vector3f& dpdv,
                        const Normal3f& dndu, const Normal3f& dndv, Float time,
                        const Shape* shape );

    void SetShadingGeometry( const Vector3f& dpdus, const Vector3f& dpdvs, const Normal3f& dndus,
                             const Normal3f& dndvs, bool orientationisAuthoritative );
};
}
#endif /* interaction_hpp */
