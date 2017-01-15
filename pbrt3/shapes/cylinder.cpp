//
//  cylinder.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/14/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "cylinder.hpp"
#include "efloat.hpp"
#include "interaction.hpp"

namespace pbrt {

Cylinder::Cylinder( const Transform* o2w, const Transform* w2o, bool ro, Float rad, Float z0,
                    Float z1, Float pm )
: Shape{ o2w, w2o, ro },
  radius{ rad },
  zMin{ std::min( z0, z1 ) },
  zMax{ std::max( z0, z1 ) },
  phiMax{ Radians( Clamp( pm, 0.f, 360.f ) ) }
{
}

Bounds3f Cylinder::ObjectBound() const
{
    Point3f p1 = Point3f( -radius, -radius, zMin );
    Point3f p2 = Point3f( radius, radius, zMax );
    return Bounds3f( p1, p2 );
}

bool Cylinder::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                          bool testAlphaTexture ) const
{
    Float phi;
    Point3f pHit;

    // transform ray to object space
    Vector3f oErr, dErr;
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute quadratic cylinder coefficients:
    EFloat A = r.d.x * r.d.x + r.d.y * r.d.y;
    EFloat B = 2 * ( r.d.x * r.o.x + r.d.y * r.o.y );
    EFloat C = r.o.x * r.o.x + r.o.y * r.o.y - radius * radius;

    // solve quadratic equation for t values
    EFloat t0, t1;
    if ( !Quadratic( A, B, C, &t0, &t1 ) )
        return false;
    if ( t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0 )
        return false;
    EFloat tShapeHit = t0;
    if ( tShapeHit.LowerBound() <= 0 ) {
        tShapeHit = t1;
        if ( tShapeHit.UpperBound() > ray.tMax )
            return false;
    }

    // compute cylinder hit point and phi
    pHit = ray( static_cast< Float >( tShapeHit ) );
    phi = atan2f( pHit.y, pHit.x );
    if ( phi < 0. )
        phi += 2.f * Pi;

    // test cylinder intersection against clipping parameters
    if ( pHit.z < zMin || pHit.z > zMax || phi > phiMax ) {
        if ( tShapeHit == t1 )
            return false;
        tShapeHit = t1;
        if ( static_cast< Float >( t1 ) > r.tMax )
            return false;
        // compute cylinder hit point and phi:
        pHit = ray( static_cast< Float >( tShapeHit ) );
        phi = atan2f( pHit.y, pHit.x );
        if ( phi < 0 )
            phi += 2.f * Pi;
        if ( pHit.z < zMin || pHit.z > zMax || phi > phiMax )
            return false;
    }

    // find parametric representation of cylinder hit
    Float u = phi / phiMax;
    Float v = ( pHit.z - zMin ) / ( zMax - zMin );
    // dp/du & dp/dv:
    Vector3f dpdu{ -phiMax * pHit.y, phiMax * pHit.x, 0 };
    Vector3f dpdv{ 0, 0, zMax - zMin };

    // dn/du & dn/dv:
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f{ pHit.x, pHit.y, 0 };
    Vector3f d2Pduv{ 0, 0, 0 }, d2Pdvv{ 0, 0, 0 };
    // compute coefficients for fundamental forms
    Float E = Dot( dpdu, dpdu );
    Float F = Dot( dpdu, dpdv );
    Float G = Dot( dpdv, dpdv );
    Vector3f N = Normalize( Cross( dpdu, dpdv ) );
    Float e = Dot( N, d2Pduu );
    Float f = Dot( N, d2Pduv );
    Float g = Dot( N, d2Pdvv );
    // compute error bounds for sphere intersection
    Float invEGF2 = static_cast< Float >( 1 ) / ( E * G - F * F );
    Normal3f dndu =
      Normal3f{ ( f * F - e * G ) * invEGF2 * dpdu + ( e * G - g * E ) * invEGF2 * dpdv };
    Normal3f dndv =
      Normal3f{ ( g * F - f * G ) * invEGF2 * dpdu + ( f * F - g * E ) * invEGF2 * dpdv };

    // initialize SurfaceInteraction from parametric information
    Vector3f pError;
    *isect = ( *ObjectToWorld )( SurfaceInteraction{ pHit, pError, Point2f{ u, v }, -ray.d, dpdu,
                                                     dpdv, dndu, dndv, ray.time, this } );

    // update tHit for quadric intersection
    *tHit = static_cast< Float >( tShapeHit );
    return true;
}

bool Cylinder::IntersectP( const Ray& r, bool testAlphaTexture ) const
{
    Float phi;
    Point3f pHit;

    // transform Ray to object space

    // compute quadratic sphere coefficients

    // solve quadratic equation for t values

    // compute sphere hit position and phi

    // test sphere intersection against clipping parameters

    return true;
}
}
