//
//  trianglemesh.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/19/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "trianglemesh.hpp"
#include "interaction.hpp"

namespace pbrt {

TriangleMesh::TriangleMesh( const Transform& ObjectToWorld, int nTriangles,
                            const int* vertexIndices, int nVertices, const Point3f* P,
                            const Vector3f* S, const Normal3f* N, const Point2f* UV,
                            const std::shared_ptr< Texture< Float > >& alphaMask )
: nTriangles{ nTriangles },
  nVertices{ nVertices },
  vertexIndices{ vertexIndices, vertexIndices + 3 * nTriangles },
  alphaMask{ alphaMask }
{
    p.reset( new Point3f[ nVertices ] );
    for ( auto i = 0; i < nVertices; ++i )
        p[ i ] = ObjectToWorld( P[ i ] );

    if ( UV ) {
        uv.reset( new Point2f[ nVertices ] );
        memcpy( uv.get(), UV, nVertices * sizeof( Point2f ) );
    }
    if ( N ) {
        n.reset( new Normal3f[ nVertices ] );
        for ( int i = 0; i < nVertices; ++i )
            n[ i ] = ObjectToWorld( N[ i ] );
    }
    if ( S ) {
        s.reset( new Vector3f[ nVertices ] );
        for ( int i = 0; i < nVertices; ++i )
            s[ i ] = ObjectToWorld( S[ i ] );
    }
}

Triangle::Triangle( const Transform* ObjectToWorld, const Transform* WorldToObject,
                    bool reverseOrientation, const std::shared_ptr< TriangleMesh >& mesh,
                    int triNumber )
: Shape{ ObjectToWorld, WorldToObject, reverseOrientation },
  mesh{ mesh },
  v{ &mesh->vertexIndices[ 3 * triNumber ] }
{
}

Bounds3f Triangle::ObjectBound() const
{
    const Point3f& p0 = mesh->p[ v[ 0 ] ];
    const Point3f& p1 = mesh->p[ v[ 1 ] ];
    const Point3f& p2 = mesh->p[ v[ 2 ] ];
    return Union( Bounds3f( ( *WorldToObject )( p0 ), ( *WorldToObject )( p1 ) ),
                  ( *WorldToObject )( p2 ) );
}

Bounds3f Triangle::WorldBound() const
{
    const Point3f& p0 = mesh->p[ v[ 0 ] ];
    const Point3f& p1 = mesh->p[ v[ 1 ] ];
    const Point3f& p2 = mesh->p[ v[ 2 ] ];
    return Union( Bounds3f( p0, p1 ), p2 );
}

bool Triangle::Intersect( const Ray& ray, Float* tHit, SurfaceInteraction* isect,
                          bool testAlphaTexture ) const
{
    // Get triangle verteices in p0, p1, and p2
    const Point3f& p0 = mesh->p[ v[ 0 ] ];
    const Point3f& p1 = mesh->p[ v[ 1 ] ];
    const Point3f& p2 = mesh->p[ v[ 2 ] ];

    // perform ray-triangle intersection test
    //      transform triangle vertices to ray coordinate space
    //          translate vertices based on ray origin
    Point3f p0t = p0 - Vector3f( ray.o );
    Point3f p1t = p1 - Vector3f( ray.o );
    Point3f p2t = p2 - Vector3f( ray.o );

    //          permute components of triangle vertices and ray direction
    int kz = MaxDimension( Abs( ray.d ) );
    int kx = kz + 1;
    if ( kx == 3 )
        kx = 0;
    int ky = kx + 1;
    if ( ky == 3 )
        ky = 0;
    Vector3f d = Permute( ray.d, kx, ky, kz );
    p0t = Permute( p0t, kx, ky, kz );
    p1t = Permute( p1t, kx, ky, kz );
    p2t = Permute( p2t, kx, ky, kz );

    //          apply shear transformation to translated vertex positions
    Float Sx = -d.x / d.z;
    Float Sy = -d.y / d.z;
    Float Sz = -1.f / d.z;
    p0t.x += Sx * p0t.z;
    p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z;
    p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z;
    p2t.y += Sy * p2t.z;

    //      compute edge function coefficients e0, e1, e2
    Float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    Float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    Float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    //      fall back to double-precision test at triangle edges -- recompute e0, e1, and e2 with
    //      doubles if any of them is == 0

    //      perform triangle edge and determinant tests
    if ( ( e0 < 0 || e1 < 0 || e2 < 0 ) && ( e0 > 0 || e1 > 0 || e2 > 0 ) )
        return false;
    Float det = e0 + e1 + e2;
    if ( det == 0 )
        return false;

    //      compute scaled hit distance to triangle and test against ray t range
    p0t.z *= Sz;
    p1t.z *= Sz;
    p2t.z *= Sz;
    Float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if ( det < 0 && ( tScaled >= 0 || tScaled < ray.tMax * det ) )
        return false;
    else if ( det > 0 && ( tScaled <= 0 || tScaled > ray.tMax * det ) )
        return false;

    //      compute barycentric coordinates and t value for triangle intersection
    Float invDet = 1 / det;
    Float b0 = e0 * invDet;
    Float b1 = e1 * invDet;
    Float b2 = e2 * invDet;
    Float t = tScaled * invDet;

    //      ensure that computed triangle t is conservatively greater than zero

    // compute triangle partial derivatives
    Vector3f dpdu, dpdv;
    Point2f uv[ 3 ];
    GetUVs( uv );
    // compute deltas for triangle partial derivatives:
    Vector2f duv02 = uv[ 0 ] - uv[ 2 ], duv12 = uv[ 1 ] - uv[ 2 ];
    Vector3f dp02 = p0 - p2, dp12 = p1 - p2;

    Float determinant = duv02[ 0 ] * duv12[ 1 ] - duv02[ 1 ] * duv12[ 0 ];
    if ( determinant == 0 ) {
        // handle zero determinant for triangle partial derivative matrix
        CoordinateSystem( Normalize( Cross( p2 - p0, p1 - p0 ) ), &dpdu, &dpdv );
    } else {
        Float invdet = 1 / determinant;
        dpdu = ( duv12[ 1 ] * dp02 - duv02[ 1 ] * dp12 ) * invdet;
        dpdv = ( -duv12[ 0 ] * dp02 - duv02[ 0 ] * dp12 ) * invdet;
    }

    // compute error bounds for triangle intersection

    // interpolate (u, v) parametric coordinates and hit point
    Point3f pHit = b0 * p0 + b1 * p1 + b2 * p2;
    Point2f uvHit = b0 * uv[ 0 ] + b1 * uv[ 1 ] + b2 * uv[ 2 ];

    // test intersection against alpha texture (if present)
    if ( testAlphaTexture && mesh->alphaMask ) {
        SurfaceInteraction isectLocal{ pHit,
                                       Vector3f{ 0, 0, 0 },
                                       uvHit,
                                       Vector3f{ 0, 0, 0 },
                                       dpdu,
                                       dpdv,
                                       Normal3f{ 0, 0, 0 },
                                       Normal3f{ 0, 0, 0 },
                                       ray.time,
                                       this };
        // if ( mesh->alphaMask->Evaluate(isectLocal) == 0 ) return false; // TODO: after Texture is
        // done
    }

    // fill in SurfaceInteraction from triangle hit
    Vector3f pError;
    *isect = SurfaceInteraction{
        pHit,     pError, uvHit, -ray.d, dpdu, dpdv, Normal3f{ 0, 0, 0 }, Normal3f{ 0, 0, 0 },
        ray.time, this
    };
    // override surface normal in isect for triangle
    isect->n = isect->shading.n = Normal3f{ Normalize( Cross( dp02, dp12 ) ) };
    if ( mesh->n || mesh->s ) {
        // initialize Triangle shading geometry
    }
    // ensure correct orientation of the geometric normal
    if ( mesh->n )
        isect->n = FaceForward( isect->n, isect->shading.n );
    else if ( reverseOrientation ^ transformSwapsHandedness )
        isect->n = isect->shading.n = -isect->n;

    *tHit = t;
    return true;
}

bool Triangle::IntersectP( const Ray& ray, bool testAlphaTexture ) const { return true; }

std::vector< std::shared_ptr< Shape > > Triangle::CreateTriangleMesh(
  const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
  int nTriangles, const int& vertexIndices, int nVertices, const Point3f* p, const Vector3f* s,
  const Normal3f* n, const Point2f* uv, const std::shared_ptr< Texture< Float > >& alphaMask )
{
    auto mesh = std::make_shared< TriangleMesh >( *ObjectToWorld, nTriangles, vertexIndices,
                                                  nVertices, p, s, n, uv, alphaMask );
    std::vector< std::shared_ptr< Shape > > tris;
    for ( auto i = 0; i < nTriangles; ++i ) {
        tris.emplace_back( std::make_shared< Triangle >( ObjectToWorld, WorldToObject,
                                                         reverseOrientation, mesh, i ) );
    }
    return tris;
}

void Triangle::GetUVs( Point2f uv[ 3 ] ) const
{
    if ( mesh->uv ) {
        uv[ 0 ] = mesh->uv[ v[ 0 ] ];
        uv[ 1 ] = mesh->uv[ v[ 1 ] ];
        uv[ 2 ] = mesh->uv[ v[ 2 ] ];
    } else {
        uv[ 0 ] = Point2f{ 0, 0 };
        uv[ 1 ] = Point2f{ 1, 0 };
        uv[ 2 ] = Point2f{ 1, 1 };
    }
}

} /* namespace pbrt */
