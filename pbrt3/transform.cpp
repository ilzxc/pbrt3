//
//  transform.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/6/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "transform.hpp"

using namespace pbrt;

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

Transform Transform::Translate( const Vector3f& delta ) const
{
    Matrix4x4 m( 1, 0, 0, delta.x, 0, 1, 0, delta.y, 0, 0, 1, delta.z, 0, 0, 0, 1 );
    Matrix4x4 mInv( 1, 0, 0, -delta.x, 0, 1, 0, -delta.y, 0, 0, 1, -delta.z, 0, 0, 0, 1 );
    return Transform( m, mInv );
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

bool Transform::SwapsHandedness() const
{
    Float det = m.m[ 0 ][ 0 ] * ( m.m[ 1 ][ 1 ] * m.m[ 2 ][ 2 ] - m.m[ 1 ][ 2 ] * m.m[ 2 ][ 1 ] ) -
                m.m[ 0 ][ 1 ] * ( m.m[ 1 ][ 0 ] * m.m[ 2 ][ 2 ] - m.m[ 1 ][ 2 ] * m.m[ 2 ][ 0 ] ) +
                m.m[ 0 ][ 2 ] * ( m.m[ 1 ][ 0 ] * m.m[ 2 ][ 1 ] - m.m[ 1 ][ 1 ] * m.m[ 2 ][ 0 ] );
    return det < 0;
}
