//
//  transform.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/6/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef transform_hpp
#define transform_hpp

#include "geometry.hpp"
#include "pbrt.hpp"
#include "quaternion.hpp"
#include "stringprint.hpp"

namespace pbrt {

struct Matrix4x4
{
    Float m[ 4 ][ 4 ];

    Matrix4x4()
    {
        m[ 0 ][ 0 ] = m[ 1 ][ 1 ] = m[ 2 ][ 2 ] = m[ 3 ][ 3 ] = 1.f;
        m[ 0 ][ 1 ] = m[ 0 ][ 2 ] = m[ 0 ][ 3 ] = m[ 1 ][ 0 ] = m[ 1 ][ 2 ] = m[ 1 ][ 3 ] =
          m[ 2 ][ 0 ] = m[ 2 ][ 1 ] = m[ 2 ][ 3 ] = m[ 3 ][ 0 ] = m[ 3 ][ 1 ] = m[ 3 ][ 2 ] = 0.f;
    }

    Matrix4x4( const Float mat[ 4 ][ 4 ] );
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

    friend Matrix4x4 Transpose( const Matrix4x4& m )
    {
        return Matrix4x4( m.m[ 0 ][ 0 ], m.m[ 1 ][ 0 ], m.m[ 2 ][ 0 ], m.m[ 3 ][ 0 ], m.m[ 0 ][ 1 ],
                          m.m[ 1 ][ 1 ], m.m[ 2 ][ 1 ], m.m[ 3 ][ 1 ], m.m[ 0 ][ 2 ], m.m[ 1 ][ 2 ],
                          m.m[ 2 ][ 2 ], m.m[ 3 ][ 2 ], m.m[ 0 ][ 3 ], m.m[ 1 ][ 3 ], m.m[ 2 ][ 3 ],
                          m.m[ 3 ][ 3 ] );
    }

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
            fprintf( f, " ]\n" );
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

    friend Matrix4x4 Inverse( const Matrix4x4& m )
    {
        int indxc[ 4 ], indxr[ 4 ];
        int ipiv[ 4 ] = { 0, 0, 0, 0 };
        Float minv[ 4 ][ 4 ];
        memcpy( minv, m.m, 4 * 4 * sizeof( Float ) );
        for ( int i = 0; i < 4; i++ ) {
            int irow = 0, icol = 0;
            Float big = 0.f;
            // Choose pivot
            for ( int j = 0; j < 4; j++ ) {
                if ( ipiv[ j ] != 1 ) {
                    for ( int k = 0; k < 4; k++ ) {
                        if ( ipiv[ k ] == 0 ) {
                            if ( std::abs( minv[ j ][ k ] ) >= big ) {
                                big = Float( std::abs( minv[ j ][ k ] ) );
                                irow = j;
                                icol = k;
                            }
                        } else if ( ipiv[ k ] > 1 )
                            Error( "Singular matrix in MatrixInvert" );
                    }
                }
            }
            ++ipiv[ icol ];
            // Swap rows _irow_ and _icol_ for pivot
            if ( irow != icol ) {
                for ( int k = 0; k < 4; ++k )
                    std::swap( minv[ irow ][ k ], minv[ icol ][ k ] );
            }
            indxr[ i ] = irow;
            indxc[ i ] = icol;
            if ( minv[ icol ][ icol ] == 0.f )
                Error( "Singular matrix in MatrixInvert" );

            // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
            Float pivinv = 1. / minv[ icol ][ icol ];
            minv[ icol ][ icol ] = 1.;
            for ( int j = 0; j < 4; j++ )
                minv[ icol ][ j ] *= pivinv;

            // Subtract this row from others to zero out their columns
            for ( int j = 0; j < 4; j++ ) {
                if ( j != icol ) {
                    Float save = minv[ j ][ icol ];
                    minv[ j ][ icol ] = 0;
                    for ( int k = 0; k < 4; k++ )
                        minv[ j ][ k ] -= minv[ icol ][ k ] * save;
                }
            }
        }
        // Swap columns to reflect permutation
        for ( int j = 3; j >= 0; j-- ) {
            if ( indxr[ j ] != indxc[ j ] ) {
                for ( int k = 0; k < 4; k++ )
                    std::swap( minv[ k ][ indxr[ j ] ], minv[ k ][ indxc[ j ] ] );
            }
        }
        return Matrix4x4( minv );
    }

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

    friend Transform Inverse( const Transform& t ) { return Transform( t.mInv, t.m ); }
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

    Transform Scale( Float x, Float y, Float z ) const;
    bool HasScale() const;
    Transform RotateX( Float theta );
    Transform RotateY( Float theta );
    Transform RotateZ( Float theta );
    Transform Rotate( Float theta, const Vector3f& axis );
    Transform LookAt( const Point3f& pos, const Point3f& look, const Vector3f& up ) const;

    // Transform Inline Functions
    template < typename T > inline Point3< T > operator()( const Point3< T >& p ) const
    {
        T x = p.x, y = p.y, z = p.z;
        T xp = m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z + m.m[ 0 ][ 3 ];
        T yp = m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z + m.m[ 1 ][ 3 ];
        T zp = m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z + m.m[ 2 ][ 3 ];
        T wp = m.m[ 3 ][ 0 ] * x + m.m[ 3 ][ 1 ] * y + m.m[ 3 ][ 2 ] * z + m.m[ 3 ][ 3 ];
        CHECK_NE( wp, 0 );
        if ( wp == 1 )
            return Point3< T >( xp, yp, zp );
        else
            return Point3< T >( xp, yp, zp ) / wp;
    }

    template < typename T > inline Vector3< T > operator()( const Vector3< T >& v ) const
    {
        T x = v.x, y = v.y, z = v.z;
        return Vector3< T >( m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z,
                             m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z,
                             m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z );
    }

    template < typename T > inline Normal3< T > operator()( const Normal3< T >& n ) const
    {
        T x = n.x, y = n.y, z = n.z;
        return Normal3< T >( mInv.m[ 0 ][ 0 ] * x + mInv.m[ 1 ][ 0 ] * y + mInv.m[ 2 ][ 0 ] * z,
                             mInv.m[ 0 ][ 1 ] * x + mInv.m[ 1 ][ 1 ] * y + mInv.m[ 2 ][ 1 ] * z,
                             mInv.m[ 0 ][ 2 ] * x + mInv.m[ 1 ][ 2 ] * y + mInv.m[ 2 ][ 2 ] * z );
    }

    inline Ray operator()( const Ray& r ) const
    {
        Vector3f oError;
        Point3f o = ( *this )( r.o, &oError );
        Vector3f d = ( *this )( r.d );
        // Offset ray origin to edge of error bounds and compute _tMax_
        Float lengthSquared = d.LengthSquared();
        Float tMax = r.tMax;
        if ( lengthSquared > 0 ) {
            Float dt = Dot( Abs( d ), oError ) / lengthSquared;
            o += d * dt;
            tMax -= dt;
        }
        return Ray( o, d, tMax, r.time, r.medium );
    }

    inline RayDifferential operator()( const RayDifferential& r ) const
    {
        Ray tr = ( *this )( Ray( r ) );
        RayDifferential ret( tr.o, tr.d, tr.tMax, tr.time, tr.medium );
        ret.hasDifferentials = r.hasDifferentials;
        ret.rxOrigin = ( *this )( r.rxOrigin );
        ret.ryOrigin = ( *this )( r.ryOrigin );
        ret.rxDirection = ( *this )( r.rxDirection );
        ret.ryDirection = ( *this )( r.ryDirection );
        return ret;
    }

    template < typename T >
    inline Point3< T > operator()( const Point3< T >& p, Vector3< T >* pError ) const
    {
        T x = p.x, y = p.y, z = p.z;
        // Compute transformed coordinates from point _pt_
        T xp = m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z + m.m[ 0 ][ 3 ];
        T yp = m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z + m.m[ 1 ][ 3 ];
        T zp = m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z + m.m[ 2 ][ 3 ];
        T wp = m.m[ 3 ][ 0 ] * x + m.m[ 3 ][ 1 ] * y + m.m[ 3 ][ 2 ] * z + m.m[ 3 ][ 3 ];

        // Compute absolute error for transformed point
        T xAbsSum = ( std::abs( m.m[ 0 ][ 0 ] * x ) + std::abs( m.m[ 0 ][ 1 ] * y ) +
                      std::abs( m.m[ 0 ][ 2 ] * z ) + std::abs( m.m[ 0 ][ 3 ] ) );
        T yAbsSum = ( std::abs( m.m[ 1 ][ 0 ] * x ) + std::abs( m.m[ 1 ][ 1 ] * y ) +
                      std::abs( m.m[ 1 ][ 2 ] * z ) + std::abs( m.m[ 1 ][ 3 ] ) );
        T zAbsSum = ( std::abs( m.m[ 2 ][ 0 ] * x ) + std::abs( m.m[ 2 ][ 1 ] * y ) +
                      std::abs( m.m[ 2 ][ 2 ] * z ) + std::abs( m.m[ 2 ][ 3 ] ) );
        *pError = gamma( 3 ) * Vector3< T >( xAbsSum, yAbsSum, zAbsSum );
        CHECK_NE( wp, 0 );
        if ( wp == 1 )
            return Point3< T >( xp, yp, zp );
        else
            return Point3< T >( xp, yp, zp ) / wp;
    }

    template < typename T >
    inline Point3< T > operator()( const Point3< T >& pt, const Vector3< T >& ptError,
                                   Vector3< T >* absError ) const
    {
        T x = pt.x, y = pt.y, z = pt.z;
        T xp = m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z + m.m[ 0 ][ 3 ];
        T yp = m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z + m.m[ 1 ][ 3 ];
        T zp = m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z + m.m[ 2 ][ 3 ];
        T wp = m.m[ 3 ][ 0 ] * x + m.m[ 3 ][ 1 ] * y + m.m[ 3 ][ 2 ] * z + m.m[ 3 ][ 3 ];
        absError->x = ( gamma( 3 ) + ( T )1 ) * ( std::abs( m.m[ 0 ][ 0 ] ) * ptError.x +
                                                  std::abs( m.m[ 0 ][ 1 ] ) * ptError.y +
                                                  std::abs( m.m[ 0 ][ 2 ] ) * ptError.z ) +
                      gamma( 3 ) * ( std::abs( m.m[ 0 ][ 0 ] * x ) + std::abs( m.m[ 0 ][ 1 ] * y ) +
                                     std::abs( m.m[ 0 ][ 2 ] * z ) + std::abs( m.m[ 0 ][ 3 ] ) );
        absError->y = ( gamma( 3 ) + ( T )1 ) * ( std::abs( m.m[ 1 ][ 0 ] ) * ptError.x +
                                                  std::abs( m.m[ 1 ][ 1 ] ) * ptError.y +
                                                  std::abs( m.m[ 1 ][ 2 ] ) * ptError.z ) +
                      gamma( 3 ) * ( std::abs( m.m[ 1 ][ 0 ] * x ) + std::abs( m.m[ 1 ][ 1 ] * y ) +
                                     std::abs( m.m[ 1 ][ 2 ] * z ) + std::abs( m.m[ 1 ][ 3 ] ) );
        absError->z = ( gamma( 3 ) + ( T )1 ) * ( std::abs( m.m[ 2 ][ 0 ] ) * ptError.x +
                                                  std::abs( m.m[ 2 ][ 1 ] ) * ptError.y +
                                                  std::abs( m.m[ 2 ][ 2 ] ) * ptError.z ) +
                      gamma( 3 ) * ( std::abs( m.m[ 2 ][ 0 ] * x ) + std::abs( m.m[ 2 ][ 1 ] * y ) +
                                     std::abs( m.m[ 2 ][ 2 ] * z ) + std::abs( m.m[ 2 ][ 3 ] ) );
        CHECK_NE( wp, 0 );
        if ( wp == 1. )
            return Point3< T >( xp, yp, zp );
        else
            return Point3< T >( xp, yp, zp ) / wp;
    }

    template < typename T >
    inline Vector3< T > operator()( const Vector3< T >& v, Vector3< T >* absError ) const
    {
        T x = v.x, y = v.y, z = v.z;
        absError->x =
          gamma( 3 ) * ( std::abs( m.m[ 0 ][ 0 ] * v.x ) + std::abs( m.m[ 0 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 0 ][ 2 ] * v.z ) );
        absError->y =
          gamma( 3 ) * ( std::abs( m.m[ 1 ][ 0 ] * v.x ) + std::abs( m.m[ 1 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 1 ][ 2 ] * v.z ) );
        absError->z =
          gamma( 3 ) * ( std::abs( m.m[ 2 ][ 0 ] * v.x ) + std::abs( m.m[ 2 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 2 ][ 2 ] * v.z ) );
        return Vector3< T >( m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z,
                             m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z,
                             m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z );
    }

    template < typename T >
    inline Vector3< T > operator()( const Vector3< T >& v, const Vector3< T >& vError,
                                    Vector3< T >* absError ) const
    {
        T x = v.x, y = v.y, z = v.z;
        absError->x =
          ( gamma( 3 ) + ( T )1 ) *
            ( std::abs( m.m[ 0 ][ 0 ] ) * vError.x + std::abs( m.m[ 0 ][ 1 ] ) * vError.y +
              std::abs( m.m[ 0 ][ 2 ] ) * vError.z ) +
          gamma( 3 ) * ( std::abs( m.m[ 0 ][ 0 ] * v.x ) + std::abs( m.m[ 0 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 0 ][ 2 ] * v.z ) );
        absError->y =
          ( gamma( 3 ) + ( T )1 ) *
            ( std::abs( m.m[ 1 ][ 0 ] ) * vError.x + std::abs( m.m[ 1 ][ 1 ] ) * vError.y +
              std::abs( m.m[ 1 ][ 2 ] ) * vError.z ) +
          gamma( 3 ) * ( std::abs( m.m[ 1 ][ 0 ] * v.x ) + std::abs( m.m[ 1 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 1 ][ 2 ] * v.z ) );
        absError->z =
          ( gamma( 3 ) + ( T )1 ) *
            ( std::abs( m.m[ 2 ][ 0 ] ) * vError.x + std::abs( m.m[ 2 ][ 1 ] ) * vError.y +
              std::abs( m.m[ 2 ][ 2 ] ) * vError.z ) +
          gamma( 3 ) * ( std::abs( m.m[ 2 ][ 0 ] * v.x ) + std::abs( m.m[ 2 ][ 1 ] * v.y ) +
                         std::abs( m.m[ 2 ][ 2 ] * v.z ) );
        return Vector3< T >( m.m[ 0 ][ 0 ] * x + m.m[ 0 ][ 1 ] * y + m.m[ 0 ][ 2 ] * z,
                             m.m[ 1 ][ 0 ] * x + m.m[ 1 ][ 1 ] * y + m.m[ 1 ][ 2 ] * z,
                             m.m[ 2 ][ 0 ] * x + m.m[ 2 ][ 1 ] * y + m.m[ 2 ][ 2 ] * z );
    }

    inline Ray operator()( const Ray& r, Vector3f* oError, Vector3f* dError ) const
    {
        Point3f o = ( *this )( r.o, oError );
        Vector3f d = ( *this )( r.d, dError );
        Float tMax = r.tMax;
        Float lengthSquared = d.LengthSquared();
        if ( lengthSquared > 0 ) {
            Float dt = Dot( Abs( d ), *oError ) / lengthSquared;
            o += d * dt;
            tMax -= dt;
        }
        return Ray( o, d, tMax, r.time, r.medium );
    }

    inline Ray operator()( const Ray& r, const Vector3f& oErrorIn, const Vector3f& dErrorIn,
                           Vector3f* oErrorOut, Vector3f* dErrorOut ) const
    {
        Point3f o = ( *this )( r.o, oErrorIn, oErrorOut );
        Vector3f d = ( *this )( r.d, dErrorIn, dErrorOut );
        Float tMax = r.tMax;
        Float lengthSquared = d.LengthSquared();
        if ( lengthSquared > 0 ) {
            Float dt = Dot( Abs( d ), *oErrorOut ) / lengthSquared;
            o += d * dt;
            //        tMax -= dt;
        }
        return Ray( o, d, tMax, r.time, r.medium );
    }

    Bounds3f operator()( const Bounds3f& b ) const;
    SurfaceInteraction operator()( const SurfaceInteraction& si ) const;

    Transform operator*( const Transform& t2 ) const;
    bool operator==( const Transform& t2 ) const;
    bool operator!=( const Transform& t2 ) const;

    bool SwapsHandedness() const;

  private:
    Matrix4x4 m, mInv;
    friend struct Quaternion;
    friend class AnimatedTransform;
};

struct Interval
{
    Float low, high;

    Interval( Float v ) : low{ v }, high{ v } {}
    Interval( Float v0, Float v1 ) : low{ v0 }, high{ v1 } {}

    Interval operator+( const Interval& i ) const { return Interval( low + i.low, high + i.high ); }

    Interval operator-( const Interval& i ) const { return Interval( low - i.high, high - i.low ); }

    Interval operator*( const Interval& i ) const
    {
        return Interval( std::min( std::min( low * i.low, high * i.low ),
                                   std::min( low * i.high, high * i.high ) ),
                         std::max( std::max( low * i.low, high * i.low ),
                                   std::max( low * i.high, high * i.high ) ) );
    }
};

inline Interval Sin( const Interval& i )
{
    Float sinLow = std::sin( i.low ), sinHigh = std::sin( i.high );
    if ( sinLow > sinHigh )
        std::swap( sinLow, sinHigh );
    if ( i.low < Pi / 2 && i.high > Pi / 2 )
        sinHigh = 1.;
    if ( i.low < ( 3.f / 2.f ) * Pi && i.high > ( 3.f / 2.f ) * Pi )
        sinLow = -1.;
    return Interval( sinLow, sinHigh );
}

inline Interval Cos( const Interval& i )
{
    Float cosLow = std::cos( i.low ), cosHigh = std::cos( i.high );
    if ( cosLow > cosHigh )
        std::swap( cosLow, cosHigh );
    if ( i.low < Pi / 2 && i.high > Pi / 2 )
        cosHigh = 1.;
    if ( i.low < ( 3.f / 2.f ) * Pi && i.high > ( 3.f / 2.f ) * Pi )
        cosLow = -1.;
    return Interval( cosLow, cosHigh );
}

void IntervalFindZeros( Float c1, Float c2, Float c3, Float c4, Float c5, Float theta,
                        Interval tInterval, Float* zeros, int* zeroCount, int depth = 8 )
{
    // evaluate motion derivative in interval form, return if no zeros:
    Interval range =
      Interval( c1 ) +
      ( Interval( c2 ) + Interval( c3 ) * tInterval ) * Cos( Interval( 2 * theta ) * tInterval ) +
      ( Interval( c4 ) + Interval( c5 ) * tInterval ) * Sin( Interval( 2 * theta ) * tInterval );
    if ( range.low > 0. || range.high < 0. || range.low == range.high )
        return;
    if ( depth > 0 ) {
        // split tInterval and check both resulting intervalsx
        Float mid = ( tInterval.low + tInterval.high ) * .5f;
        IntervalFindZeros( c1, c2, c3, c4, c5, theta, Interval( tInterval.low, mid ), zeros,
                           zeroCount, depth - 1 );
        IntervalFindZeros( c1, c2, c3, c4, c5, theta, Interval( mid, tInterval.high ), zeros,
                           zeroCount, depth - 1 );
    } else {
        // use newton's method to refine zero
        Float tNewton = ( tInterval.low + tInterval.high ) * .5f;
        for ( auto i = 0; i < 4; ++i ) {
            Float fNewton = c1 + ( c2 + c3 * tNewton ) * std::cos( 2.f * theta * tNewton ) +
                            ( c4 + c5 * tNewton ) * std::sin( 2.f * theta * tNewton );
            Float fPrimeNewton =
              ( c3 + 2 * ( c4 + c5 * tNewton ) * theta ) * std::cos( 2.f * tNewton * theta ) +
              ( c5 - 2 * ( c2 + c3 * tNewton ) * theta ) * std::sin( 2.f * tNewton * theta );
            if ( fNewton == 0 || fPrimeNewton == 0 )
                break;
            tNewton = tNewton - fNewton / fPrimeNewton;
        }
        zeros[ *zeroCount ] = tNewton;
        ++( *zeroCount );
    }
}

class AnimatedTransform {
  public:
    AnimatedTransform( const Transform* startTransform, Float startTime,
                       const Transform* endTransform, Float endTime );

    void Decompose( const Matrix4x4& m, Vector3f* T, Quaternion* R, Matrix4x4* S );
    void Interpolate( Float time, Transform* t ) const;

    Ray operator()( const Ray& r ) const;
    RayDifferential operator()( const RayDifferential& r ) const;
    Point3f operator()( const Point3f& p ) const;
    Vector3f operator()( const Vector3f& v ) const;

    Bounds3f MotionBounds( const Bounds3f& b ) const;
    Bounds3f BoundPointMotion( const Point3f& p ) const;

  private:
    struct DerivativeTerm
    {
        Float kc, kx, ky, kz;

        DerivativeTerm() {}
        DerivativeTerm( Float c, Float x, Float y, Float z ) : kc{ c }, kx{ x }, ky{ y }, kz{ z } {}

        Float Eval( const Point3f& p ) const { return kc + kx * p.x + ky * p.y + kz * p.z; }
    };

    const Transform* startTransform;
    const Transform* endTransform;
    const Float startTime, endTime;
    Vector3f T[ 2 ];
    Quaternion R[ 2 ];
    Matrix4x4 S[ 2 ];
    DerivativeTerm c1[ 3 ], c2[ 3 ], c3[ 3 ], c4[ 4 ], c5[ 3 ];
    const bool actuallyAnimated;
    bool hasRotation;
};

} /* namespace pbrt */
#endif /* transform_hpp */
