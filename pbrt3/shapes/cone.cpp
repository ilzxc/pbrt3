//
//  cone.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "cone.hpp"
#include "efloat.hpp"
#include "interaction.hpp"

namespace pbrt {

Cone::Cone( const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
            Float height, Float radius, Float phiMax )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  height{ height },
  radius{ radius },
  phiMax{ phiMax }
{
}

Bounds3f Cone::ObjectBound() const
{
    return Bounds3f{ Point3f{ -radius, -radius, 0 }, Point3f{ radius, radius, height } };
}

bool Cone::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                      bool testAlphaTexture ) const
{
    Vector3f oErr, dErr;
    // transform ray to object space
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute quadric cone coefficients
    EFloat ox( ray.o.x, oErr.x ), oy( ray.o.y, oErr.y ), oz( ray.o.z, oErr.z );
    EFloat dx( ray.d.x, dErr.x ), dy( ray.d.y, dErr.y ), dz( ray.d.z, dErr.z );
    EFloat k = EFloat( radius ) / EFloat( height );
    k = k * k;
    EFloat a = dx * dx + dy * dy - k * dz * dz;
    EFloat b = 2 * ( dx * ox + dy * oy - k * dz * ( oz - height ) );
    EFloat c = ox * ox + oy * oy - k * ( oz - height ) * ( oz - height );

    // solve quadratic equation for t values
    EFloat t0, t1;
    if ( !Quadratic( a, b, c, &t0, &t1 ) )
        return false;

    // check t0 and t1 for nearest intersection
    if ( t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0 )
        return false;
    EFloat tShapeHit = t0;
    if ( tShapeHit.LowerBound() <= 0 ) {
        tShapeHit = t1;
        if ( tShapeHit.UpperBound() > ray.tMax )
            return false;
    }

    // compute cone inverse mapping (?)
    Point3f pHit = ray( static_cast< Float >( tShapeHit ) );
    Float phi = std::atan2( pHit.y, pHit.x );
    if ( phi < 0 )
        phi += 2 * Pi;

    // test against clipping parameters
    if ( pHit.z < 0 || pHit.z > height || phi > phiMax ) {
        if ( tShapeHit == t1 )
            return false;
        tShapeHit = t1;
        if ( t1.UpperBound() > ray.tMax )
            return false;
        pHit = ray( static_cast< Float >( tShapeHit ) );
        phi = std::atan2( pHit.y, pHit.x );
        if ( phi < 0 )
            phi += 2 * Pi;
        if ( pHit.z < 0 || pHit.z > height || phi > phiMax )
            return false;
    }

    // find parametric representation of cone hit
    Float u = phi / phiMax;
    Float v = pHit.z / height;

    // dpdu and dpdv:
    Vector3f dpdu{ -phiMax * pHit.y, phiMax * pHit.x, 0 };
    Float inv = static_cast< Float >( 1 ) / ( 1 - v );
    Vector3f dpdv{ -pHit.x * inv, -pHit.y * inv, height };

    Vector3f d2Pduu = Vector3f{ pHit.x, pHit.y, 0 } * ( -phiMax * phiMax );
    Vector3f d2Pduv = Vector3f{ pHit.y, -pHit.x, 0 } * ( phiMax * inv );
    Vector3f d2Pdvv{ 0, 0, 0 };

    // compute coefficients for fundamental forms
    Float E = Dot( dpdv, dpdu );
    Float F = Dot( dpdu, dpdv );
    Float G = Dot( dpdv, dpdv );
    Vector3f N = Normalize( Cross( dpdu, dpdv ) );
    Float e = Dot( N, d2Pduu );
    Float f = Dot( N, d2Pduv );
    Float g = Dot( N, d2Pdvv );

    // compute dndu and dndv from fundamental form coefficients
    Float invEGF2 = static_cast< Float >( 1 ) / ( E * G - F * F );
    Normal3f dndu{ ( f * F - e * G ) * invEGF2 * dpdu + ( e * F - F * E ) * invEGF2 * dpdv };
    Normal3f dndv{ ( g * F - F * G ) * invEGF2 * dpdu + ( f * F - g * E ) * invEGF2 * dpdv };

    // Error bounds computation
    EFloat px = ox + tShapeHit * dx;
    EFloat py = oy + tShapeHit * dy;
    EFloat pz = oz + tShapeHit * dz;
    Vector3f pError{ px.GetAbsoluteError(), py.GetAbsoluteError(), pz.GetAbsoluteError() };

    // initialize SurfaceIntersection from parametric information
    *isect = ( *ObjectToWorld )( SurfaceInteraction{ pHit, pError, Point2f{ u, v }, -ray.d, dpdu,
                                                     dpdv, dndu, dndv, ray.time, this } );

    // update tHit for quadric intersection
    *tHit = static_cast< Float >( tShapeHit );

    return true;
}

bool Cone::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

} /* namespace pbrt */
