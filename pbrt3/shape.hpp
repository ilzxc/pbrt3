//
//  shape.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/7/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef shape_hpp
#define shape_hpp

#include "geometry.hpp"
#include "pbrt.hpp"

namespace pbrt {

struct Shape
{
    Shape( const Transform* objectToWorld, const Transform* worldToObject,
           bool reverseOrientation );

    virtual ~Shape();
    virtual Bounds3f ObjectBound() const = 0;
    Bounds3f WorldBound() const;
    virtual bool Intersect( const Ray& ray, Float* tHit, SurfaceInteraction* isect,
                            bool testAlphaTexture = true ) const = 0;
    virtual bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const
    {
        Float tHit = ray.tMax;
        SurfaceInteraction isect;
        return Intersect( ray, &tHit, &isect, testAlphaTexture );
    }

    const Transform* ObjectToWorld;
    const Transform* WorldToObject;
    const bool reverseOrientation;
    const bool transformSwapsHandidness;
};

} /* namespace pbrt */
#endif /* shape_hpp */
