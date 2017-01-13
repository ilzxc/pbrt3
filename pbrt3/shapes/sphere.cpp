//
//  sphere.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/7/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "sphere.hpp"
#include "efloat.hpp"
#include "geometry.hpp"
#include "interaction.hpp"
#include "transform.hpp"

namespace pbrt {

Sphere::Sphere( const Transform* ObjectToWorld, const Transform* WorldToObject,
                bool reverseOrientation, Float radius, Float zMin, Float zMax, Float phiMax )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  radius{ radius },
  zMin{ Clamp( std::min( zMin, zMax ), -radius, radius ) },
  zMax{ Clamp( std::max( zMin, zMax ), -radius, radius ) },
  thetaMin{ std::acos( Clamp( zMin / radius, -1, 1 ) ) },
  thetaMax{ std::acos( Clamp( zMax / radius, -1, 1 ) ) },
  phiMax{ Radians( Clamp( phiMax, 0, 360 ) ) }
{
}

Bounds3f Sphere::ObjectBound() const
{
    return Bounds3f{ Point3f( -radius, -radius, zMin ), Point3f( radius, radius, zMax ) };
}

bool Sphere::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                        bool testAlphaTexture ) const
{
    Float phi;
    Point3f pHit;

    // transform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute quadratic sphere coefficients
    EFloat ox{ ray.o.x, oErr.x }, oy{ ray.o.y, oErr.y }, oz{ ray.o.z, oErr.z };
    EFloat dx{ ray.d.x, oErr.x }, dy{ ray.d.y, oErr.y }, dz{ ray.d.z, oErr.z };

    EFloat a = dx * dx + dy * dy + dz * dz;
    EFloat b = 2 * ( dx * ox + dy * oy + dz * oz );
    EFloat c = ox * ox + oy * oy + oz * oz - EFloat( radius ) * EFloat( radius );

    // solve quadratic equation for t values
    EFloat t0, t1;
    if ( !Quadratic( a, b, c, &t0, &t1 ) )
        return false;

    if ( t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0 )
        return false;
    EFloat tShapeHit = t0;
    if ( tShapeHit.LowerBound() <= 0 ) {
        tShapeHit = t1;
        if ( tShapeHit.UpperBound() > ray.tMax )
            return false;
    }

    // compute sphere hit position and phi
    pHit = ray( static_cast< Float >( tShapeHit ) );
    // refine sphere intersection point
    if ( pHit.x == 0 & pHit.y == 0 )
        pHit.x = 1e-5f * radius;

    phi = std::atan2( pHit.y, pHit.x );
    if ( phi < 0 )
        phi += 2 * Pi;

    // test sphere intersection against clipping parameters
    if ( ( zMin > -radius && pHit.z < zMin ) || ( zMax < radius && pHit.z > zMax ) ||
         phi > phiMax ) {
        if ( tShapeHit == t1 )
            return false;
        if ( t1.UpperBound() > ray.tMax )
            return false;
        tShapeHit = t1;
        // compute sphere hit position and phi
        pHit = ray( static_cast< Float >( tShapeHit ) );
        // refine sphere intersection point
        if ( pHit.x == 0 & pHit.y == 0 )
            pHit.x = 1e-5f * radius;

        phi = std::atan2( pHit.y, pHit.x );
        if ( phi < 0 )
            phi += 2 * Pi;
        if ( ( zMin > -radius && pHit.z < zMin ) || ( zMax < radius && pHit.z > zMax ) ||
             phi > phiMax )
            return false;
    }

    // find parameteric representation of sphere hit
    Float u = phi / phiMax;
    Float theta = std::acos( Clamp( pHit.z / radius, -1, 1 ) );
    Float v = ( theta - thetaMin ) / ( thetaMax - thetaMin );
    // compute sphere dp/du and dp/dv partial derivatives
    Float zRadius = std::sqrt( pHit.x * pHit.x + pHit.y * pHit.y );
    Float invZRadius = static_cast< Float >( 1 ) / zRadius;
    Float cosPhi = pHit.x * invZRadius;
    Float sinPhi = pHit.y * invZRadius;
    Vector3f dpdu( -phiMax * pHit.y, phiMax * pHit.x, 0 );
    Vector3f dpdv = ( thetaMax - thetaMin ) *
                    Vector3f( pHit.z * cosPhi, pHit.z * sinPhi, -radius * std::sin( theta ) );
    // compute sphere dn/du and dn/dv partial derivatives
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f( pHit.x, pHit.y, 0 );
    Vector3f d2Pduv = ( thetaMax - thetaMin ) * pHit.z * phiMax * Vector3f( -sinPhi, cosPhi, 0 );
    Vector3f d2Pdvv =
      -( thetaMax - thetaMin ) * ( thetaMax - thetaMin ) * Vector3f( pHit.x, pHit.y, pHit.z );
    // compute coefficients for fundamental forms
    // compute dn/du and dn/dv from fundamental forms coefficients
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
    *isect = ( *ObjectToWorld )( SurfaceInteraction{ pHit, pError, Point2f( u, v ), -ray.d, dpdu,
                                                     dpdv, dndu, dndv, ray.time, this } );
    // update tHit for quadric intersection
    *tHit = static_cast< Float >( tShapeHit );
    return true;
}

bool Sphere::IntersectP( const Ray& r, bool testAlphaTexture ) const
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

} /* namespace pbrt */
