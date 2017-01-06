//
//  transform.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/6/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef transform_hpp
#define transform_hpp

#include "pbrt.hpp"

using Float = float;

struct Matrix4x4
{
    Float m[ 4 ][ 4 ];

    Matrix4x4()
    {
        m[ 0 ][ 0 ] = m[ 1 ][ 1 ] = m[ 2 ][ 2 ] = m[ 3 ][ 3 ] = 1.f;
        m[ 0 ][ 1 ] = m[ 0 ][ 2 ] = m[ 0 ][ 3 ] = m[ 1 ][ 0 ] = m[ 1 ][ 2 ] = m[ 1 ][ 3 ] =
          m[ 2 ][ 0 ] = m[ 2 ][ 1 ] = m[ 2 ][ 3 ] = m[ 3 ][ 0 ] = m[ 3 ][ 1 ] = m[ 3 ][ 2 ] = 0.f;
    }

    Matrix4x4( Float mat[ 4 ][ 4 ] );
    Matrix4x4( Float t00, Float t01, Float t02, Float t03, Float t10, Float t11, Float t12,
               Float t13, Float t20, Float t21, Float t22, Float t23, Float t30, Float t31,
               Float t32, Float t33 );

    bool operator==( const Matrix4x4& m2 ) const
    {
        for ( auto i = 0; i < 4; ++i )
            for ( auto j = 0; j < 4; ++j )
                if ( m[ i ][ j ] != m2.m[ i ][ j ] )
                    return false;
        return true;
    }

    bool operator!=( const Matrix4x4& m2 ) const
    {
        for ( auto i = 0; i < 4; ++i )
            for ( auto j = 0; j < 4; ++j )
                if ( m[ i ][ j ] != m2.m[ i ][ j ] )
                    return true;
        return false;
    }

    friend Matrix4x4 Transpose( const Matrix4x4& );
    void Print( FILE* f ) const
    {
        fprintf( f, "[ " );
        for ( auto i = 0; i < 4; ++i ) {
            fprintf( f, "  [ " );
            for ( auto j = 0; j < 4; ++j ) {
                fprintf( f, "%f", m[ i ][ j ] );
                if ( j != 3 )
                    fprintf( f, ", " );
            }
            printf( f, " ]\n" );
        }
    }

    static Matrix4x4 Mul( const Matrix4x4& m1, const Matrix4x4& m2 )
    {
        Matrix4x4 r;
        for ( auto i = 0; i < 4; ++i )
            for ( auto j = 0; j < 4; ++j )
                r.m[ i ][ j ] = m1.m[ i ][ 0 ] * m2.m[ 0 ][ j ] + m1.m[ i ][ 1 ] * m2.m[ 1 ][ j ] +
                                m1.m[ i ][ 2 ] * m2.m[ 2 ][ j ] + m1.m[ i ][ 3 ] * m2.m[ 3 ][ j ];
        return r;
    }

    friend Matrix4x4 Inverse( const Matrix4x4& );

    friend std::ostream& operator<<( std::ostream& os, const Matrix4x4& m )
    {
        os << StringPrintf( "[ [ %f, %f, %f, %f ] "
                            "  [ %f, %f, %f, %f ] "
                            "  [ %f, %f, %f, %f ] "
                            "  [ %f, %f, %f, %f ] ]",
                            m.m[ 0 ][ 0 ], m.m[ 0 ][ 1 ], m.m[ 0 ][ 2 ], m.m[ 0 ][ 3 ],
                            m.m[ 1 ][ 0 ], m.m[ 1 ][ 1 ], m.m[ 1 ][ 2 ], m.m[ 1 ][ 3 ],
                            m.m[ 2 ][ 0 ], m.m[ 2 ][ 1 ], m.m[ 2 ][ 2 ], m.m[ 2 ][ 3 ],
                            m.m[ 3 ][ 0 ], m.m[ 3 ][ 1 ], m.m[ 3 ][ 2 ], m.m[ 3 ][ 3 ] );
        return os;
    }
};

class Transform {
  public:
    Transform() {}
    Transform( const Float mat[ 4 ][ 4 ] ) : m{ mat }, mInv{ Inverse( m ) } {}
    Transform( const Matrix4x4& m ) : m{ m }, mInv{ Inverse( m ) } {}
    Transform( const Matrix4x4& m, const Matrix4x4& mInv ) : m{ m }, mInv{ mInv } {}

    friend Transform Inverse( const Transform& t ) { return Transform( mInv, m ); }
    friend Transform Transpose( const Transform& t )
    {
        return Transform( Transpose( t.m ), Transpose( t.mInv ) );
    }

    Transform Translate( const Vector3f& delta ) const
    {
        Matrix4x4 m( 1, 0, 0, delta.x, 0, 1, 0, delta.y, 0, 0, 1, delta.z, 0, 0, 0, 1 );
        Matrix4x4 mInv( 1, 0, 0, -delta.x, 0, 1, 0, -delta.y, 0, 0, 1, -delta.z, 0, 0, 0, 1 );
        return Transform( m, mInv );
    }

    Transform Scale( Float x, Float y, Float z ) const
    {
        Matrix4x4 m( x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1 );
        Matrix4x4 mInv = ( 1 / x, 0, 0, 0, 0, 1 / x, 0, 0, 0, 0, 1 / x, 0, 0, 0, 0, 1 );
        return Transform( m, mInv );
    }

    bool HasScale() const
    {
        Float la2 = ( *this )( Vector3f( 1, 0, 0 ) ).LengthSquared();
        Float lb2 = ( *this )( Vector3f( 0, 1, 0 ) ).LengthSquared();
        Float lc2 = ( *this )( Vector3f( 0, 0, 1 ) ).LengthSquared();
#define NOT_ONE( x ) ( ( x ) < .999f || ( x ) > 1.001f )
        return ( NOT_ONE( la2 ) || NOT_ONE( lb2 ) || NOT_ONE( lc2 ) );
#undef NOT_ONE
    }

    Transform RotateX( Float theta )
    {
        Float sinTheta = std::sin( Radians( theta ) );
        Float cosTheta = std::cos( Radians( theta ) );
        Matrix4x4 m( 1, 0, 0, 0, 0, cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0, 0, 0, 0, 1 );
        return Transform( m, Transpose( m ) );
    }

    Transform RotateY( Float theta )
    {
        Float sinTheta = std::sin( Radians( theta ) );
        Float cosTheta = std::cos( Radians( theta ) );
        Matrix4x4 m( cosTheta, 0, sinTheta, 0, 0, 1, 0, 0 - sinTheta, 0, cosTheta, 0, 0, 0, 0, 1 );
        return Transform( m, Transpose( m ) );
    }

    Transform RotateZ( Float theta )
    {
        Float sinTheta = std::sin( Radians( theta ) );
        Float cosTheta = std::cos( Radians( theta ) );
        Matrix4x4 m( cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 );
        return Transform( m, Transpose( m ) );
    }

    Transform Rotate( Float theta, const Vector3f& axis )
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

    Transform LookAt( const Point3f& pos, const Point3f& look, const Vector3f& up )
    {
        Matrix4x4 cameraToWorld;

        // fourth column:
        cameraToWorld.m[ 0 ][ 3 ] = pos.x;
        cameraToWorld.m[ 1 ][ 3 ] = pos.y;
        cameraToWorld.m[ 2 ][ 3 ] = pos.z;
        cameraToWorld.m[ 3 ][ 3 ] = 1;

        // todo: other three columns!
    }

  private:
    Matrix4x4 m, mInv;
};

#endif /* transform_hpp */
