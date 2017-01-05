//
//  geometry.h
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/4/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef geometry_h
#define geometry_h

#include <algorithm>
#include <cmath>

using Float = float;

// Vector2 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Vector2
{
    T x, y;

    Vector2() : x{ 0 }, y{ 0 } {}
    Vector2( T x, T y ) : x{ x }, y{ y }
    {
        // Assert( ! HasNaNs() );
    }

    T operator[]( int i ) const
    {
        if ( i == 0 )
            return x;
        return y;
    }

    Vector2< T > operator+( const Vector2< T >& v ) const
    {
        return Vector2< T >( x + v.x, y + v.y );
    }

    Vector2< T >& operator+=( const Vector2< T >& v )
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2< T > operator-( const Vector2< T >& v ) const
    {
        return Vector2< T >( x - v.x, y - v.y );
    }

    Vector2< T >& operator-=( const Vector2< T >& v )
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2< T > operator*( T s ) const { return Vector2< T >( s * x, s * y ); }

    Vector2< T >& operator*=( T s )
    {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2< T > operator/( T s ) const
    {
        Float inv = static_cast< Float >( 1.0 ) / s;
        return Vector2< T >( x * inv, y * inv );
    }

    Vector2< T >& operator/=( T s )
    {
        Float inv = static_cast< Float >( 1.0 ) / s;
        x *= inv;
        y *= inv;
        return *this;
    }

    Vector2< T > operator-() const { return Vector2< T >( -x, -y ); }

    Float LengthSquared() const { return x * x + y * y; }
    Float Length() const { return std::sqrt( LengthSquared() ); }

  private:
    bool HasNaNs() const { return std::isnan( x ) || std::isnan( y ); }
};

// Numerical Ops, Vector2
template < typename T > inline Vector2< T > operator*( T s, const Vector2< T >& v )
{
    return v * s;
}

template < typename T > inline Vector2< T > Abs( const Vector2< T >& v )
{
    return Vector2< T >( std::abs( v.x ), std::abs( v.y ) );
}

template < typename T > inline Vector2< T > Dot( const Vector2< T >& v1, const Vector2< T >& v2 )
{
    return v1.x * v2.x + v1.y * v2.y;
}

template < typename T > inline Vector2< T > AbsDot( const Vector2< T >& v1, const Vector2< T >& v2 )
{
    return std::abs( Dot( v1, v2 ) );
}

template < typename T > inline Vector2< T > Normalize( const Vector2< T >& v )
{
    return v / v.Length();
}

// Miscellany, Vector2
template < typename T > T MinComponent( const Vector2< T >& v ) { return std::min( v.x, v.y ); }

template < typename T > T MaxComponent( const Vector2< T >& v ) { return std::max( v.x, v.y ); }

template < typename T > int MaxDimension( const Vector2< T >& v ) { return ( v.x > v.y ) ? 0 : 1; }

template < typename T > Vector2< T > Min( const Vector2< T >& p1, const Vector2< T >& p2 )
{
    return Vector2< T >( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ) );
}

template < typename T > Vector2< T > Max( const Vector2< T >& p1, const Vector2< T >& p2 )
{
    return Vector2< T >( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ) );
}

template < typename T > Vector2< T > Permute( const Vector2< T >& v, int x, int y )
{
    return Vector2< T >( v[ x ], v[ y ] );
}

// Vector3 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Vector3
{
    T x, y, z;

    Vector3() : x{ 0 }, y{ 0 }, z{ 0 } {}
    Vector3( T x, T y, T z ) : x{ x }, y{ y }, z{ z }
    {
        // Assert( ! HasNaNs() );
    }

    T operator[]( int i ) const
    {
        // Assert( i >= 0 && i < 3 );
        if ( i == 0 )
            return x;
        if ( i == 1 )
            return y;
        return z;
    }

    Vector3< T > operator+( const Vector3< T >& v ) const
    {
        return Vector3( x + v.x, y + v.y, z + v.z );
    }

    Vector3< T > operator+=( const Vector3< T >& v )
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3< T > operator-( const Vector3< T >& v ) const
    {
        return Vector3( x - v.x, y - v.y, z - v.z );
    }

    Vector3< T > operator-=( const Vector3< T >& v )
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3< T > operator*( T s ) const { return Vector3( s * x, s * y, s * z ); }

    Vector3< T > operator*=( T s )
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vector3< T > operator/( T s ) const
    {
        Float inv = static_cast< Float >( 1. ) / s;
        return Vector3( x * inv, y * inv, z * inv );
    }

    Vector3< T > operator/=( T s )
    {
        Float inv = static_cast< Float >( 1. ) / s;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    Vector3< T > operator-() const { return Vector3< T >( -x, -y, -z ); }

    Float LengthSquared() const { return x * x + y * y + z * z; }
    Float Length() const { return std::sqrt( LengthSquared() ); }

  private:
    bool HasNaNs() const { return std::isnan( x ) || std::isnan( y ) || std::isnan( z ); }
};

// Numerical Ops, Vector3
template < typename T > inline Vector3< T > operator*( T s, const Vector3< T >& v )
{
    return v * s;
}

template < typename T > inline Vector3< T > Abs( const Vector3< T >& v )
{
    return Vector3< T >( std::abs( v.x ), std::abs( v.y ), std::abs( v.z ) );
}

template < typename T > inline Vector3< T > Dot( const Vector3< T >& v1, const Vector3< T >& v2 )
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template < typename T > inline Vector3< T > AbsDot( const Vector3< T >& v1, const Vector3< T >& v2 )
{
    return std::abs( Dot( v1, v2 ) );
}

template < typename T > inline Vector3< T > Cross( const Vector3< T >& v1, const Vector3< T >& v2 )
{
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3< T >( ( v1y * v2z ) - ( v1z * v2y ), ( v1z * v2x ) - ( v1x * v2z ),
                         ( v1x * v2y ) - ( v1y * v2x ) );
}

template < typename T > inline Vector3< T > Normalize( const Vector3< T >& v )
{
    return v / v.Length();
}

// Miscellany, Vector3
template < typename T > T MinComponent( const Vector3< T >& v )
{
    return std::min( v.x, std::min( v.y, v.z ) );
}

template < typename T > T MaxComponent( const Vector3< T >& v )
{
    return std::max( v.x, std::max( v.y, v.z ) );
}

template < typename T > int MaxDimension( const Vector3< T >& v )
{
    return ( v.x > v.y ) ? ( ( v.x > v.z ) ? 0 : 2 ) : ( ( v.y > v.z ) ? 1 : 2 );
}

template < typename T > Vector3< T > Min( const Vector3< T >& p1, const Vector3< T >& p2 )
{
    return Vector3< T >( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ), std::min( p1.z, p2.z ) );
}

template < typename T > Vector3< T > Max( const Vector3< T >& p1, const Vector3< T >& p2 )
{
    return Vector3< T >( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ), std::max( p1.z, p2.z ) );
}

template < typename T > Vector3< T > Permute( const Vector3< T >& v, int x, int y, int z )
{
    return Vector3< T >( v[ x ], v[ y ], v[ z ] );
}

template < typename T >
inline void CoordinateSystem( const Vector3< T >& v1, Vector3< T >* v2, Vector3< T >* v3 )
{
    if ( std::abs( v1.x ) > std::abs( v1.y ) )
        *v2 = Vector3< T >( -v1.z, 0, v1.x ) / std::sqrt( v1.x * v1.x + v1.z * v1.z );
    else
        *v2 = Vector3< T >( 0, v1.z, -v1.y ) / std::sqrt( v1.y * v1.y + v1.z * v1.z );
    *v3 = Cross( v1, *v2 );
}

typedef Vector2< Float > Vector2f;
typedef Vector2< int > Vector2i;
typedef Vector3< Float > Vector3f;
typedef Vector3< int > Vector3i;

// Point3 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Point3
{
    T x, y, z;

    Point3() : x{ 0 }, y{ 0 }, z{ 0 } {}
    Point3( T x, T y, T z ) : x{ x }, y{ y }, z{ z }
    {
        // Assert( ! HasNaNs() );
    }

    template < typename U >
    explicit Point3( const Point3< U >& p )
    : x{ static_cast< T >( p.x ) }, y{ static_cast< T >( p.y ) }, z{ static_cast< T >( p.z ) }
    {
        // Assert( ! HasNaNs() );
    }

    template < typename U > explicit operator Vector3< U >() const
    {
        return Vector3< U >( x, y, z );
    }

    Point3< T > operator+( const Vector3< T >& v ) const
    {
        return Point3< T >( x + v.x, y + v.y, z + v.z );
    }

    Point3< T >& operator+=( const Vector3< T >& v )
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Point3< T > operator+( const Point3< T >& p ) const
    {
        return Point3< T >( x + p.x, y + p.y, z + p.z );
    }

    Point3< T >& operator+=( const Point3< T >& p )
    {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }

    Vector3< T > operator-( const Point3< T >& p ) const
    {
        return Vector3< T >( x - p.x, y - p.y, z - p.z );
    }

    Point3< T > operator-( const Vector3< T >& v ) const
    {
        return Point3< T >( x - v.x, y - v.y, z - v.z );
    }

    Point3< T >& operator-=( const Vector3< T >& v )
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Point3< T > operator*( T s ) const { return Point3< T >( s * x, s * y, s * z ); }

    Point3< T >& operator*=( T s ) const
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

  private:
    bool HasNaNs() const { return std::isnan( x ) || std::isnan( y ) || std::isnan( z ); }
};

template < typename T > inline float Distance( const Point3< T >& p1, const Point3< T >& p2 )
{
    return ( p1 - p2 ).Length();
}

template < typename T > inline float DistanceSquared( const Point3< T >& p1, const Point3< T >& p2 )
{
    return ( p1 - p2 ).LengthSquared();
}

template < typename T > inline Point3< T > operator*( T s, const Point3< T >& p ) { return p * s; }

template < typename T > Point3< T > Lerp( Float t, const Point3< T >& p0, const Point3< T >& p1 )
{
    return ( 1 - t ) * p0 + ( p1 * t );
}

template < typename T > Point3< T > Min( const Point3< T >& p1, const Point3< T >& p2 )
{
    return Point3< T >( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ), std::min( p1.z, p2.z ) );
}

template < typename T > Point3< T > Max( const Point3< T >& p1, const Point3< T >& p2 )
{
    return Point3< T >( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ), std::max( p1.z, p2.z ) );
}

template < typename T > Point3< T > Floor( const Point3< T >& p )
{
    return Point3< T >( std::floor( p.x ), std::floor( p.y ), std::floor( p.z ) );
}

template < typename T > Point3< T > Ceil( const Point3< T >& p )
{
    return Point3< T >( std::ceil( p.x ), std::ceil( p.y ), std::ceil( p.z ) );
}

template < typename T > Point3< T > Abs( const Point3< T >& p )
{
    return Point3< T >( std::abs( p.x ), std::abs( p.y ), std::abs( p.z ) );
}

template < typename T > Point3< T > Permute( const Point3< T >& p, int x, int y, int z )
{
    return Point3< T >( p[ x ], p[ y ], p[ z ] );
}

// Point2 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Point2
{
    T x, y;

    Point2() : x{ 0 }, y{ 0 } {}
    Point2( T x, T y ) : x{ x }, y{ y }
    {
        // Assert( ! HasNaNs() );
    }

    explicit Point2( const Point3< T >& p ) : x{ p.x }, y{ p.y }
    {
        // Assert( ! HasNaNs() );
    }

    template < typename U >
    explicit Point2( const Point2< U >& p )
    : x{ static_cast< T >( p.x ) }, y{ static_cast< T >( p.y ) }
    {
        // Assert( ! HasNaNs() );
    }

    template < typename U > explicit operator Vector2< U >() const { return Vector2< U >( x, y ); }

    Point2< T > operator+( const Vector2< T >& v ) const { return Point2< T >( x + v.x, y + v.y ); }

    Point2< T >& operator+=( const Vector2< T >& v )
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Point2< T > operator+( const Point2< T >& p ) const { return Point2< T >( x + p.x, y + p.y ); }
    Point2< T >& operator+=( const Point2< T >& p )
    {
        x += p.x;
        y += p.y;
        return *this;
    }

    Vector2< T > operator-( const Point2< T >& p ) const
    {
        return Vector2< T >( x - p.x, y - p.y );
    }

    Point2< T > operator-( const Vector2< T >& v ) const { return Point2< T >( x - v.x, y - v.y ); }

    Point2< T >& operator-=( const Vector2< T >& v )
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Point2< T > operator*( T s ) { return Point2< T >( x * s, y * s ); }
    Point2< T >& operator*=( T s )
    {
        x *= s;
        y *= s;
        return *this;
    }

  private:
    bool HasNaNs() const { return std::isnan( x ) || std::isnan( y ); }
};

template < typename T > inline float Distance( const Point2< T >& p1, const Point2< T >& p2 )
{
    return ( p1 - p2 ).Length();
}

template < typename T > inline float DistanceSquared( const Point2< T >& p1, const Point2< T >& p2 )
{
    return ( p1 - p2 ).LengthSquared();
}

template < typename T > inline Point2< T > operator*( T s, const Point2< T >& p ) { return p * s; }

template < typename T > Point2< T > Lerp( Float t, const Point2< T >& p0, const Point2< T >& p1 )
{
    return ( 1 - t ) * p0 + ( p1 * t );
}

template < typename T > Point2< T > Min( const Point2< T >& p1, const Point2< T >& p2 )
{
    return Point2< T >( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ) );
}

template < typename T > Point2< T > Max( const Point2< T >& p1, const Point2< T >& p2 )
{
    return Point2< T >( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ) );
}

template < typename T > Point2< T > Floor( const Point2< T >& p )
{
    return Point2< T >( std::floor( p.x ), std::floor( p.y ) );
}

template < typename T > Point2< T > Ceil( const Point2< T >& p )
{
    return Point2< T >( std::ceil( p.x ), std::ceil( p.y ) );
}

template < typename T > Point2< T > Abs( const Point2< T >& p )
{
    return Point2< T >( std::abs( p.x ), std::abs( p.y ) );
}

template < typename T > Point2< T > Permute( const Point2< T >& p, int x, int y )
{
    return Point2< T >( p[ x ], p[ y ] );
}

typedef Point2< Float > Point2f;
typedef Point2< int > Point2i;
typedef Point3< Float > Point3f;
typedef Point3< int > Point3i;

#endif /* geometry_h */
