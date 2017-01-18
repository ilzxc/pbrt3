//
//  hyperboloid.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "hyperboloid.hpp"
#include "efloat.hpp"
#include "interaction.hpp"

namespace pbrt {

Hyperboloid::Hyperboloid( const Transform* ObjectToWorld, const Transform* WorldToObject,
                          bool reverseOrientation, const Point3f& point1, const Point3f& point2,
                          Float tm )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  p1{ point1 },
  p2{ point2 },
  phiMax{ Radians( Clamp( tm, 0, 360 ) ) }
{
    Float radius1 = std::sqrt( p1.x * p1.x + p1.y * p1.y + p1.z * p1.z );
    Float radius2 = std::sqrt( p2.x * p2.x + p2.y * p2.y + p2.z * p2.z );
    rMax = std::max( radius1, radius2 );
    zMin = std::min( p1.z, p2.z );
    zMax = std::max( p1.z, p2.z );
    if ( p2.z == 0 )
        std::swap( p1, p2 );

    Point3f pp = p1;
    Float xy1, xy2;
    do {
        pp += static_cast< Float >( 2 ) * ( p2 - p1 );
        xy1 = pp.x * pp.x + pp.y * pp.y;
        xy2 = p2.x * p2.x + p2.y * p2.y;
        ah = ( 1.f / xy1 - ( pp.z * pp.z ) / ( xy1 * p2.z * p2.z ) ) /
             ( 1.f - ( xy2 * pp.z * pp.z ) / ( xy1 * p2.z * p2.z ) );
        ch = ( ah * xy2 - 1 ) / ( p2.z * p2.z );
    } while ( std::isinf( ah ) || std::isnan( ah ) );
}

Bounds3f Hyperboloid::ObjectBound() const
{
    return Bounds3f{ Point3f{ -rMax, -rMax, zMin }, Point3f{ rMax, rMax, zMax } };
}

bool Hyperboloid::Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                             bool testAlphaTexture ) const
{
    Vector3f oErr, dErr;
    // transform ray to object space
    Ray ray = ( *WorldToObject )( r, &oErr, &dErr );

    // compute quadric hyperboloid coefficients
    EFloat ox( ray.o.x, oErr.x ), oy( ray.o.y, oErr.y ), oz( ray.o.z, oErr.z );
    EFloat dx( ray.d.x, dErr.x ), dy( ray.d.y, dErr.y ), dz( ray.d.z, dErr.z );
    EFloat a = ah * dx * dx + ah * dy * dy - ch * dz * dz;
    EFloat b = 2.f * ( ah * dx * ox + ah * dy * oy - ch * dz * oz );
    EFloat c = ah * ox * ox + ah * oy * oy - ch * oz * oz - 1.f;

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
    Float v = ( pHit.z - p1.z ) / ( p2.z - p1.z );
    Point3f pr = ( 1 - v ) * p1 + v * p2;
    Float phi = std::atan2( pr.x * pHit.y - pHit.x * pr.y, pHit.x * pr.x + pHit.y * pr.y );
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
        phi = ( pHit.z - p1.z ) / ( p2.z - p1.z );
        Point3f pr = ( 1 - v ) * p1 + v * p2;
        phi = std::atan2( pr.x * pHit.y - pHit.x * pr.y, pHit.x * pr.x + pHit.y * pr.y );
        if ( phi < 0 )
            phi += 2 * Pi;
        if ( pHit.z < zMin || pHit.z > zMax || phi > phiMax )
            return false;
    }

    // find parametric representation of hyperboloid hit
    Float u = phi / phiMax;

    // dpdu and dpdv:
    Float cosPhi = std::cos( phi ), sinPhi = std::sin( phi );
    Vector3f dpdu{ -phiMax * pHit.y, phiMax * pHit.x, 0 };
    Vector3f dpdv{ ( p2.x - p1.x ) * cosPhi - ( p2.y - p1.y ) * sinPhi,
                   ( p2.x - p1.x ) * sinPhi + ( p2.y - p1.y ) * cosPhi, p2.z - p1.z };
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f{ pHit.x, pHit.y, 0 };
    Vector3f d2Pduv = phiMax * Vector3f{ -dpdv.y, -dpdv.x, 0 };
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

    // error bounds computation
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

bool Hyperboloid::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

} /* namespace pbrt */
