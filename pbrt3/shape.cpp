//
//  shape.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/7/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "shape.hpp"
#include "interaction.hpp"
#include "transform.hpp"

namespace pbrt {

Shape::Shape( const Transform* objectToWorld, const Transform* worldToObject,
              bool reverseOrientation )
: ObjectToWorld{ objectToWorld },
  WorldToObject{ worldToObject },
  reverseOrientation{ reverseOrientation },
  transformSwapsHandedness{ ObjectToWorld->SwapsHandedness() }
{
}

Bounds3f Shape::WorldBound() const { return ( *ObjectToWorld )( ObjectBound() ); }

bool Shape::IntersectP( const Ray& ray, bool testAlphaTexture ) const
{
    Float tHit = ray.tMax;
    SurfaceInteraction isect;
    return Intersect( ray, &tHit, &isect, testAlphaTexture );
}

} /* namespace pbrt */
