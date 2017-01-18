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
    // transform ray to object space
    Vector3f oErr, dErr;
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute plane intersection for disk
    if ( ray.d.z == 0 )
        return false; // reject for rays parallel to the disk's plane
    Float tShapeHit = ( height - ray.o.z ) / ray.d.z;
    if ( tShapeHit <= 0 || tShapeHit >= ray.tMax )
        return false;

    // see if hit point is inside disk radii and phiMax
    Point3f pHit = ray( tShapeHit );
    Float dist2 = pHit.x * pHit.x + pHit.y * pHit.y;
    if ( dist2 > radius * radius || dist2 < innerRadius * innerRadius )
        return false;
    Float phi = std::atan2( pHit.y, pHit.x );
    if ( phi < 0 )
        phi += 2 * Pi;
    if ( phi > phiMax )
        return false;

    // find parametric representation of the disk hit
    Float u = phi / phiMax;
    Float rHit = std::sqrt( dist2 );
    Float oneMinusV = ( rHit - innerRadius ) / ( radius - innerRadius );
    Float v = 1 - oneMinusV;
    Vector3f dpdu{ -phiMax * pHit.y, phiMax * pHit.x, 0 };
    Vector3f dpdv = Vector3f{ pHit.x, pHit.y, 0 } * ( innerRadius - radius ) / rHit;
    Normal3f dndu{ 0, 0, 0 }, dndv{ 0, 0, 0 };

    // refine disk intersection point
    pHit.z = height;

    // compute error bounds for disk intersection
    Vector3f pError{ 0, 0, 0 };

    // initialize SurfaceInteraction from parametric information
    *isect = ( *ObjectToWorld )( SurfaceInteraction{ pHit, pError, Point2f{ u, v }, -ray.d, dpdu,
                                                     dpdv, dndu, dndv, ray.time, this } );
    // update tHit for quadric intersection
    *tHit = static_cast< Float >( tShapeHit );

    return true;
}

bool Disk::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

} /* namespace pbrt */
