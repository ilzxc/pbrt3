//
//  paraboloid.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "paraboloid.hpp"
#include "efloat.hpp"
#include "interaction.hpp"

namespace pbrt {

Paraboloid::Paraboloid( const Transform* ObjectToWorld, const Transform* WorldToObject,
                        bool reverseOrientation, Float radius, Float z1, Float z2, Float phiMax )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  radius{ radius },
  zMin{ std::min( z1, z2 ) },
  zMax{ std::max( z1, z2 ) },
  phiMax{ Radians( Clamp( phiMax, 0, 360 ) ) }
{
}

Bounds3f Paraboloid::ObjectBound() const
{
    return Bounds3f{ Point3f{ -radius, -radius, zMin }, Point3f{ radius, radius, zMax } };
}

bool Paraboloid::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                            bool testAlphaTexture ) const
{
    Vector3f oErr, dErr;
    // transform ray to object space
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute quadric paraboloid coefficients
    EFloat ox( ray.o.x, oErr.x ), oy( ray.o.y, oErr.y ), oz( ray.o.z, oErr.z );
    EFloat dx( ray.d.x, dErr.x ), dy( ray.d.y, dErr.y ), dz( ray.d.z, dErr.z );
    EFloat k = EFloat{ zMax } / ( EFloat{ radius } * EFloat{ radius } );
    EFloat a = k * ( dx * dx + dy * dy );
    EFloat b = 2 * k * ( dx * ox + dy * oy ) - dz;
    EFloat c = k * ( ox * ox + oy * oy ) - oz;

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
    if ( pHit.z < zMin || pHit.z > zMax || phi > phiMax ) {
        if ( tShapeHit == t1 )
            return false;
        tShapeHit = t1;
        if ( t1.UpperBound() > ray.tMax )
            return false;
        pHit = ray( static_cast< Float >( tShapeHit ) );
        phi = std::atan2( pHit.y, pHit.x );
        if ( phi < 0 )
            phi += 2 * Pi;
        if ( pHit.z < zMin || pHit.z > zMax || phi > phiMax )
            return false;
    }

    // find parametric representation of paraboloid hit
    Float zd = zMax - zMin;
    Float u = phi / phiMax;
    Float v = ( pHit.z - zMin ) / zd;

    // dpdu and dpdv:
    Vector3f dpdu{ -phiMax * pHit.y, phiMax * pHit.x, 0 };
    Vector3f dpdv = zd * Vector3f{ pHit.x / ( 2 * pHit.z ), pHit.y / ( 2 * pHit.z ), 1 };
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f{ pHit.x, pHit.y, 0 };
    Vector3f d2Pduv = phiMax * zd * Vector3f{ -dpdv.y, dpdv.x, 0 };
    Float inv = static_cast< Float >( 1.f ) / ( 4.f * pHit.z * pHit.z );
    Vector3f d2Pdvv = ( -zd * zd ) * Vector3f{ pHit.x * inv, pHit.y * inv, 0 };

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

bool Paraboloid::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

} /* namespace pbrt */
