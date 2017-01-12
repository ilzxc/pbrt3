//
//  transform.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/6/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "transform.hpp"

namespace pbrt {

Matrix4x4::Matrix4x4( const Float mat[ 4 ][ 4 ] ) { memcpy( m, mat, 16 * sizeof( Float ) ); }

Matrix4x4::Matrix4x4( Float t00, Float t01, Float t02, Float t03, Float t10, Float t11, Float t12,
                      Float t13, Float t20, Float t21, Float t22, Float t23, Float t30, Float t31,
                      Float t32, Float t33 )
{
    m[ 0 ][ 0 ] = t00;
    m[ 0 ][ 1 ] = t01;
    m[ 0 ][ 2 ] = t02;
    m[ 0 ][ 3 ] = t03;
    m[ 1 ][ 0 ] = t10;
    m[ 1 ][ 1 ] = t11;
    m[ 1 ][ 2 ] = t12;
    m[ 1 ][ 3 ] = t13;
    m[ 2 ][ 0 ] = t20;
    m[ 2 ][ 1 ] = t21;
    m[ 2 ][ 2 ] = t22;
    m[ 2 ][ 3 ] = t23;
    m[ 3 ][ 0 ] = t30;
    m[ 3 ][ 1 ] = t31;
    m[ 3 ][ 2 ] = t32;
    m[ 3 ][ 3 ] = t33;
}

Transform Transform::Scale( Float x, Float y, Float z ) const
{
    Matrix4x4 m( x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1 );
    Matrix4x4 mInv( 1 / x, 0, 0, 0, 0, 1 / x, 0, 0, 0, 0, 1 / x, 0, 0, 0, 0, 1 );
    return Transform( m, mInv );
}

bool Transform::HasScale() const
{
    Float la2 = ( *this )( Vector3f( 1, 0, 0 ) ).LengthSquared();
    Float lb2 = ( *this )( Vector3f( 0, 1, 0 ) ).LengthSquared();
    Float lc2 = ( *this )( Vector3f( 0, 0, 1 ) ).LengthSquared();
#define NOT_ONE( x ) ( ( x ) < .999f || ( x ) > 1.001f )
    return ( NOT_ONE( la2 ) || NOT_ONE( lb2 ) || NOT_ONE( lc2 ) );
#undef NOT_ONE
}

Transform Transform::RotateX( Float theta )
{
    Float sinTheta = std::sin( Radians( theta ) );
    Float cosTheta = std::cos( Radians( theta ) );
    Matrix4x4 m( 1, 0, 0, 0, 0, cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0, 0, 0, 0, 1 );
    return Transform( m, Transpose( m ) );
}

Transform Transform::RotateY( Float theta )
{
    Float sinTheta = std::sin( Radians( theta ) );
    Float cosTheta = std::cos( Radians( theta ) );
    Matrix4x4 m( cosTheta, 0, sinTheta, 0, 0, 1, 0, 0 - sinTheta, 0, cosTheta, 0, 0, 0, 0, 0, 1 );
    return Transform( m, Transpose( m ) );
}

Transform Transform::RotateZ( Float theta )
{
    Float sinTheta = std::sin( Radians( theta ) );
    Float cosTheta = std::cos( Radians( theta ) );
    Matrix4x4 m( cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 );
    return Transform( m, Transpose( m ) );
}

Transform Transform::Rotate( Float theta, const Vector3f& axis )
{
    Vector3f a = Normalize( axis );
    Float sinTheta = std::sin( Radians( theta ) );
    Float cosTheta = std::cos( Radians( theta ) );
    Matrix4x4 m;

    m.m[ 0 ][ 0 ] = a.x * a.x + ( 1 - a.x * a.x ) * cosTheta;
    m.m[ 0 ][ 1 ] = a.x * a.y * ( 1 - cosTheta ) - a.z * sinTheta;
    m.m[ 0 ][ 2 ] = a.x * a.z * ( 1 - cosTheta ) + a.y * sinTheta;
    m.m[ 0 ][ 3 ] = 0;

    m.m[ 1 ][ 0 ] = a.x * a.y * ( 1 - cosTheta ) + a.z * sinTheta;
    m.m[ 1 ][ 1 ] = a.y * a.y + ( 1 - a.y * a.y ) * cosTheta;
    m.m[ 1 ][ 2 ] = a.y * a.z * ( 1 - cosTheta ) - a.x * sinTheta;
    m.m[ 1 ][ 3 ] = 0;

    m.m[ 2 ][ 0 ] = a.x * a.z * ( 1 - cosTheta ) - a.y * sinTheta;
    m.m[ 2 ][ 1 ] = a.y * a.z * ( 1 - cosTheta ) + a.x * sinTheta;
    m.m[ 2 ][ 2 ] = a.z * a.z + ( 1 - a.z * a.z ) * cosTheta;
    m.m[ 2 ][ 3 ] = 0;

    return Transform( m, Transpose( m ) );
}

Transform Transform::LookAt( const Point3f& pos, const Point3f& look, const Vector3f& up ) const
{
    Matrix4x4 cameraToWorld;

    Vector3f dir = Normalize( look - pos );
    Vector3f left = Normalize( Cross( Normalize( up ), dir ) );
    Vector3f newUp = Cross( dir, left );

    cameraToWorld.m[ 0 ][ 0 ] = left.x;
    cameraToWorld.m[ 1 ][ 0 ] = left.y;
    cameraToWorld.m[ 2 ][ 0 ] = left.z;
    cameraToWorld.m[ 3 ][ 0 ] = 0;

    cameraToWorld.m[ 0 ][ 1 ] = newUp.x;
    cameraToWorld.m[ 1 ][ 1 ] = newUp.y;
    cameraToWorld.m[ 2 ][ 1 ] = newUp.z;
    cameraToWorld.m[ 3 ][ 1 ] = 0;

    cameraToWorld.m[ 0 ][ 2 ] = dir.x;
    cameraToWorld.m[ 1 ][ 2 ] = dir.y;
    cameraToWorld.m[ 2 ][ 2 ] = dir.z;
    cameraToWorld.m[ 3 ][ 2 ] = 0;

    cameraToWorld.m[ 0 ][ 3 ] = pos.x;
    cameraToWorld.m[ 1 ][ 3 ] = pos.y;
    cameraToWorld.m[ 2 ][ 3 ] = pos.z;
    cameraToWorld.m[ 3 ][ 3 ] = 1;

    return Transform( Inverse( cameraToWorld ), cameraToWorld );
}

Transform Transform::operator*( const Transform& t2 ) const
{
    return Transform( Matrix4x4::Mul( m, t2.m ), Matrix4x4::Mul( t2.mInv, mInv ) );
}

bool Transform::operator==( const Transform& t2 ) const
{
    return ( m == t2.m ); // && mInv == t2.mInv ); // is unnecessary
}

bool Transform::operator!=( const Transform& t2 ) const
{
    return ( m != t2.m ); // || mInv != t2.mInv ); // is unnecessary
}

bool Transform::SwapsHandedness() const
{
    Float det = m.m[ 0 ][ 0 ] * ( m.m[ 1 ][ 1 ] * m.m[ 2 ][ 2 ] - m.m[ 1 ][ 2 ] * m.m[ 2 ][ 1 ] ) -
                m.m[ 0 ][ 1 ] * ( m.m[ 1 ][ 0 ] * m.m[ 2 ][ 2 ] - m.m[ 1 ][ 2 ] * m.m[ 2 ][ 0 ] ) +
                m.m[ 0 ][ 2 ] * ( m.m[ 1 ][ 0 ] * m.m[ 2 ][ 1 ] - m.m[ 1 ][ 1 ] * m.m[ 2 ][ 0 ] );
    return det < 0;
}

AnimatedTransform::AnimatedTransform( const Transform* startTransform, Float startTime,
                                      const Transform* endTransform, Float endTime )
: startTransform{ startTransform },
  endTransform{ endTransform },
  startTime{ startTime },
  endTime{ endTime },
  actuallyAnimated{ *startTransform != *endTransform }
{
    Decompose( startTransform->m, &T[ 0 ], &R[ 0 ], &S[ 0 ] );
    Decompose( endTransform->m, &T[ 1 ], &R[ 1 ], &S[ 1 ] );
    if ( Dot( R[ 0 ], R[ 1 ] ) < 0 )
        R[ 1 ] = -R[ 1 ]; // Flip R[1] if needed to select shortest path
    hasRotation = Dot( R[ 0 ], R[ 1 ] ) < .9995f;
    // Compute terms of motion derivative function
}

void AnimatedTransform::Decompose( const Matrix4x4& m, Vector3f* T, Quaternion* Rquat,
                                   Matrix4x4* S )
{
    // extract translation T from transformation Matrix
    T->x = m.m[ 0 ][ 3 ];
    T->y = m.m[ 1 ][ 3 ];
    T->z = m.m[ 2 ][ 3 ];
    // compute new transformation matrix M without translation
    Matrix4x4 M = m;
    for ( auto i = 0; i < 3; ++i )
        M.m[ i ][ 3 ] = M.m[ 3 ][ i ] = 0.f;
    M.m[ 3 ][ 3 ] = 1.f;
    // compute new rotation R from transformation matrix
    Float norm;
    int count = 0;
    Matrix4x4 R = M;
    do {
        // compute next matrix Rnext in series
        Matrix4x4 Rnext;
        Matrix4x4 Rit = Inverse( Transpose( R ) );
        for ( auto i = 0; i < 4; ++i )
            for ( auto j = 0; j < 4; ++j )
                Rnext.m[ i ][ j ] = 0.5f * ( R.m[ i ][ j ] + Rit.m[ i ][ j ] );
        // compute norm of difference between Rm and Rnext
        norm = 0;
        for ( auto i = 0; i < 3; ++i ) {
            Float n = std::abs( R.m[ i ][ 0 ] - Rnext.m[ i ][ 0 ] ) +
                      std::abs( R.m[ i ][ 1 ] - Rnext.m[ i ][ 1 ] ) +
                      std::abs( R.m[ i ][ 2 ] - Rnext.m[ i ][ 2 ] );
            norm = std::max( norm, n );
        }
        R = Rnext;
    } while ( norm > .0001 && ++count < 100 );
    *Rquat = Quaternion{ R };

    // compute scale S using rotation & original matrix
    *S = Matrix4x4::Mul( Inverse( R ), M );
}

void AnimatedTransform::Interpolate( Float time, Transform* t ) const
{
    // Handle boundary conditions for matrix interpolation
    if ( !actuallyAnimated || time <= startTime ) {
        *t = *startTransform;
        return;
    }
    if ( time >= endTime ) {
        *t = *endTransform;
        return;
    }
    Float dt = ( time - startTime ) / ( endTime - startTime );
    // Interpolate translation at dt
    Vector3f trans = ( 1 - dt ) * T[ 0 ] + dt * T[ 1 ];
    // Interpolate rotation at dt
    Quaternion rotate = Slerp( dt, R[ 0 ], R[ 1 ] );
    // Interpolate scale at dt
    Matrix4x4 scale;
    for ( auto i = 0; i < 3; ++i )
        for ( auto j = 0; j < 3; ++j )
            scale.m[ i ][ j ] = Lerp( dt, S[ 0 ].m[ i ][ j ], S[ 1 ].m[ i ][ j ] );
    // compute interpolated matrix as product of interpolated components
    *t = t->Translate( trans ) * rotate.ToTransform() * Transform( scale );
}

Ray AnimatedTransform::operator()( const Ray& r ) const { return Ray{}; }

RayDifferential AnimatedTransform::operator()( const RayDifferential& r ) const
{
    return RayDifferential{};
}

Point3f AnimatedTransform::operator()( const Point3f& p ) const { return Point3f{}; }

Vector3f AnimatedTransform::operator()( const Vector3f& v ) const { return Vector3f{}; }

Bounds3f AnimatedTransform::MotionBounds( const Bounds3f& b ) const
{
    if ( !actuallyAnimated )
        return ( *startTransform )( b );
    if ( !hasRotation )
        return Union( ( *startTransform )( b ), ( *endTransform )( b ) );
    // otherwise return motion bounds accounting for animated rotateion:
    Bounds3f bounds;
    for ( auto corner = 0; corner < 8; ++corner )
        bounds = Union( bounds, BoundPointMotion( b.Corner( corner ) ) );
    return bounds;
}

Bounds3f AnimatedTransform::BoundPointMotion( const Point3f& p ) const
{
    Bounds3f bounds( ( *startTransform )( p ), ( *endTransform )( p ) );
    Float cosTheta = Dot( R[ 0 ], R[ 1 ] );
    Float theta = std::acos( Clamp( cosTheta, -1, 1 ) );
    for ( auto c = 0; c < 3; ++c ) {
        // find any motion derivative zeros for the component c
        Float zeros[ 4 ];
        int nZeros = 0;
        IntervalFindZeros( c1[ c ].Eval( p ), c2[ c ].Eval( p ), c3[ c ].Eval( p ),
                           c4[ c ].Eval( p ), c5[ c ].Eval( p ), theta, Interval( 0., 1. ), zeros,
                           &nZeros );
        // expand bounding box for any motion derivative zeros found
        for ( auto i = 0; i < nZeros; ++i ) {
            Point3f pz = ( *this )( Lerp( zeros[ i ], startTime, endTime ), p );
            bounds = Union( bounds, pz );
        }
    }
    return bounds;
}

} /* namespace pbrt */
