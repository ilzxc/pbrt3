//
//  Disk.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "disk.hpp"
#include "geometry.hpp"
#include "interaction.hpp"

namespace pbrt {

Disk::Disk( const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
            Float height, Float radius, Float innerRadius, Float phiMax )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  height{ height },
  radius{ radius },
  innerRadius{ innerRadius },
  phiMax{ Clamp( phiMax, 0, 360 ) }
{
}

Bounds3f Disk::ObjectBound() const
{
    return Bounds3f{ Point3f{ -radius, -radius, height }, Point3f{ radius, radius, height } };
}

bool Disk::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                      bool testAlphaTexture ) const
{
    return true;
}

bool Disk::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

} /* namespace pbrt */
