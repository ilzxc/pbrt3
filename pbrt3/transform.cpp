//
//  transform.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/6/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "transform.hpp"

using namespace pbrt;

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
