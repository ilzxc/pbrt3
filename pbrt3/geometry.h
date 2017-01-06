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
Float Infinity = std::numeric_limits< Float >::infinity();

inline Float Lerp( Float t, Float v1, Float v2 ) { return ( 1 - t ) * v1 + t * v2; }

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

template < typename T > inline T Dot( const Vector2< T >& v1, const Vector2< T >& v2 )
{
    return v1.x * v2.x + v1.y * v2.y;
}

template < typename T > inline T AbsDot( const Vector2< T >& v1, const Vector2< T >& v2 )
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

// forward-declaration of Normal3 (see below)
template < typename T > struct Normal3;

template < typename T > struct Vector3
{
    T x, y, z;

    Vector3() : x{ 0 }, y{ 0 }, z{ 0 } {}
    Vector3( T x, T y, T z ) : x{ x }, y{ y }, z{ z }
    {
        // Assert( ! HasNaNs() );
    }

    Vector3( const Normal3< T >& n );

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

template < typename T > inline T Dot( const Vector3< T >& v1, const Vector3< T >& v2 )
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template < typename T > inline T AbsDot( const Vector3< T >& v1, const Vector3< T >& v2 )
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

    bool operator==( const Point3< T >& p ) const { return x == p.x && y == p.y && z == p.z; }

    bool operator!=( const Point3< T >& p ) const { return x != p.x || y != p.y || z != p.z; }

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

    bool operator==( const Point2< T >& p ) const { return x == p.x && y == p.y; }

    bool operator!=( const Point2< T >& p ) const { return x != p.x || y != p.y; }

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

// Normal  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Normal3
{
    T x, y, z;
    Normal3() : x{ 0 }, y{ 0 }, z{ 0 } {}
    Normal3( T x, T y, T z ) : x{ x }, y{ y }, z{ z }
    {
        // Assert( ! HasNaNs() );
    }
    explicit Normal3( const Vector3< T >& v ) : x{ v.x }, y{ v.y }, z{ v.z }
    {
        // Assert( ! HasNaNs() );
    }

    Normal3< T > operator+( const Normal3< T >& n ) const
    {
        return Normal3( x + n.x, y + n.y, z + n.z );
    }

    Normal3< T >& operator+=( const Normal3< T >& n )
    {
        x += n.x;
        y += n.y;
        z += n.z;
        return *this;
    }

    Normal3< T > operator-( const Normal3< T >& n ) const
    {
        return Normal3( x - n.x, y - n.y, z - n.z );
    }

    Normal3< T >& operator-=( const Normal3< T >& n )
    {
        x -= n.x;
        y -= n.y;
        z -= n.z;
        return *this;
    }

    Normal3< T > operator-() { return Normal3( -x, -y, -z ); }

    Normal3< T > operator*( T s ) const { return Normal3( s * x, s * y, s * z ); }

    Normal3< T >& operator*=( T s )
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Float LengthSquared() const { return x * x + y * y + z * z; }

    Float Length() const { return std::sqrt( LengthSquared() ); }
};

template < typename T > inline Normal3< T > operator*( T s, Normal3< T >& n ) { return n * s; }

template < typename T > inline Normal3< T > Normalize( const Normal3< T >& n )
{
    Float inv = static_cast< Float >( 1.0 ) / n.Length();
    return n * inv;
}

template < typename T > inline Normal3< T > Abs( const Normal3< T >& n )
{
    return Normal3< T >( std::abs( n.x ), std::abs( n.y ), std::abs( n.z ) );
}

template < typename T > inline T Dot( const Normal3< T >& n1, const Normal3< T >& n2 )
{
    return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

template < typename T > inline T Dot( const Normal3< T >& n, const Vector3< T >& v )
{
    return n.x * v.x + n.y * v.y + n.z * v.z;
}

template < typename T > inline T Dot( const Vector3< T >& v, const Normal3< T >& n )
{
    return v.x * n.x + v.y * n.y + v.z * n.z;
}

template < typename T > inline T AbsDot( const Normal3< T >& n1, const Normal3< T >& n2 )
{
    return std::abs( Dot( n1, n2 ) );
}

template < typename T > inline T AbsDot( const Normal3< T >& n, const Vector3< T >& v )
{
    return std::abs( Dot( n, v ) );
}

template < typename T > inline T AbsDot( const Vector3< T >& v, const Normal3< T >& n )
{
    return std::abs( Dot( v, n ) );
}

template < typename T >
inline Vector3< T >::Vector3( const Normal3< T >& n ) : x{ n.x }, y{ n.y }, z{ n.z }
{
    // Assert( ! n.HasNaNs() );
}

template < typename T >
inline Normal3< T > FaceForward( const Normal3< T >& n, const Vector3< T >& v )
{
    return ( Dot( n, v ) < 0.f ) ? -n : n;
}

template < typename T >
inline Normal3< T > FaceForward( const Vector3< T >& v, const Normal3< T >& n )
{
    return ( Dot( n, v ) < 0.f ) ? -n : n;
}

template < typename T >
inline Vector3< T > FaceForward( const Vector3< T >& v1, const Vector3< T >& v2 )
{
    return ( Dot( v1, v2 ) < 0.f ) ? -v1 : v1;
}

template < typename T >
inline Normal3< T > FaceForward( const Normal3< T >& n1, const Normal3< T >& n2 )
{
    return ( Dot( n1, n2 ) < 0.f ) ? -n1 : n1;
}

// Typedefs of common geometry types:
typedef Vector2< Float > Vector2f;
typedef Vector2< int > Vector2i;
typedef Vector3< Float > Vector3f;
typedef Vector3< int > Vector3i;
typedef Point2< Float > Point2f;
typedef Point2< int > Point2i;
typedef Point3< Float > Point3f;
typedef Point3< int > Point3i;
typedef Normal3< Float > Normal3f;

// Ray  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Medium; // forward-declared for now

struct Ray
{
    Point3f o;
    Vector3f d;
    mutable Float tMax;
    Float time;
    const Medium* medium;

    Ray() : tMax{ Infinity }, time{ 0.f }, medium{ nullptr } {}
    Ray( const Point3f& o, const Vector3f& d, Float tMax = Infinity, Float time = 0.f,
         const Medium* medium = nullptr )
    : o{ o }, d{ d }, tMax{ tMax }, time{ time }, medium{ medium }
    {
    }

    Point3f operator()( Float t ) const { return o + d * t; }
};

struct RayDifferential : public Ray
{
    bool hasDifferentials;
    Point3f rxOrigin, ryOrigin;
    Vector3f rxDirection, ryDirection;

    RayDifferential() : hasDifferentials{ false } {}
    RayDifferential( const Point3f& o, const Vector3f& d, Float tMax = Infinity, Float time = 0.f,
                     const Medium* medium = nullptr )
    : Ray{ o, d, tMax, time, medium }, hasDifferentials{ false }
    {
    }
    RayDifferential( const Ray& ray ) : Ray{ ray }, hasDifferentials{ false } {}

    void ScaleDifferentials( Float s )
    {
        rxOrigin = o + ( rxOrigin - o ) * s;
        ryOrigin = o + ( ryOrigin - o ) * s;
        rxDirection = d + ( rxDirection - d ) * s;
        ryDirection = d + ( ryDirection - d ) * s;
    }
};

// Bounds  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < typename T > struct Bounds2
{
    Point2< T > pMin, pMax;
    Bounds2()
    {
        T minNum = std::numeric_limits< T >::lowest();
        T maxNum = std::numeric_limits< T >::max();
        pMin = Point2< T >( maxNum, maxNum );
        pMax = Point2< T >( minNum, minNum );
    }
    Bounds2( const Point2< T >& p ) : pMin{ p }, pMax{ p } {}
    Bounds2( const Point2< T >& p1, const Point2< T >& p2 )
    : pMin{ std::min( p1.x, p2.x ), std::min( p1.y, p2.y ) },
      pMax{ std::max( p1.x, p2.x ), std::max( p1.y, p2.y ) }
    {
    }

    const Point2< T >& operator[]( int i ) const
    {
        if ( i == 0 )
            return pMin;
        return pMax;
    }

    Point2< T >& operator[]( int i )
    {
        if ( i == 0 )
            return pMin;
        return pMax;
    }

    Point2< T > Corner( int corner ) const
    {
        return Point2< T >( ( *this )[ ( corner & 1 ) ].x, ( *this )[ ( corner & 2 ) ? 1 : 0 ].y );
    }

    Vector3< T > Diagonal() const { return pMax - pMin; }

    T SurfaceArea() const
    {
        auto d = Diagonal();
        return d.x * d.y;
    }

    T Volume() const
    {
        auto d = Diagonal();
        return d.x * d.y;
    }

    int MaximumExtent() const
    {
        auto d = Diagonal();
        if ( d.x > d.y )
            return 0;
        return 1;
    }

    Point2< T > Lerp( const Point2f& t ) const
    {
        return Point2< T >( Lerp( t.x, pMin.x, pMax.x ), Lerp( t.y, pMin.y, pMax.y ) );
    }

    Vector2< T > Offset( const Point2< T >& p ) const
    {
        auto o = p - pMin;
        if ( pMax.x > pMin.x )
            o.x /= pMax.x - pMin.x;
        if ( pMax.y > pMin.y )
            o.y /= pMax.y - pMin.y;
        return o;
    }

    void BoundingSphere( Point3< T >* center, Float* radius ) const
    {
        *center = ( pMin + pMax ) / 2;
        *radius = Inside( *center, *this ) ? Distance( *center, pMax ) : 0;
    }
};

template < typename T > Bounds2< T > Union( const Bounds2< T >& b, const Point2< T >& p )
{
    return Bounds2< T >( Point2< T >( std::min( b.pmin.x, p.x ), std::min( b.pMin.y, p.y ) ),
                         Point2< T >( std::max( b.pMax.x, p.x ), std::max( b.pMax.y, p.y ) ) );
}

template < typename T > Bounds2< T > Union( const Bounds2< T >& b1, const Bounds2< T >& b2 )
{
    return Bounds2< T >(
      Point2< T >( std::min( b1.pmin.x, b2.pMin.x ), std::min( b1.pMin.y, b2.pMin.y ) ),
      Point2< T >( std::max( b1.pMax.x, b2.pMax.x ), std::max( b1.pMax.y, b2.pMax.y ) ) );
}

template < typename T > bool Overlaps( const Bounds2< T >& b1, const Bounds2< T >& b2 )
{
    bool x = ( b1.pMax.x >= b2.pMin.x ) && ( b1.pMin.x <= b2.pMax.x );
    bool y = ( b1.pMax.y >= b2.pMin.y ) && ( b1.pMin.y <= b2.pMax.y );
    return ( x && y );
}

template < typename T > bool Inside( const Point2< T >& p, const Bounds2< T >& b )
{
    return ( p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y );
}

template < typename T > bool InsideExclusive( const Point2< T >& p, const Bounds2< T >& b )
{
    return ( p.x >= b.pMin.x && p.x < b.pMax.x && p.y >= b.pMin.y && p.y < b.pMax.y );
}

template < typename T, typename U > inline Bounds2< T > Expand( const Bounds2< T >& b, U delta )
{
    return Bounds2< T >( b.pMin - Vector2< T >( delta, delta ),
                         b.pMax + Vector2< T >( delta, delta ) );
}

template < typename T > struct Bounds3
{
    Point3< T > pMin, pMax;

    Bounds3()
    {
        T minNum = std::numeric_limits< T >::lowest();
        T maxNum = std::numeric_limits< T >::max();
        pMin = Point3< T >( maxNum, maxNum, maxNum );
        pMax = Point3< T >( minNum, minNum, minNum );
    }
    Bounds3( const Point3< T >& p ) : pMin{ p }, pMax{ p } {}
    Bounds3( const Point3< T >& p1, const Point3< T >& p2 )
    : pMin{ std::min( p1.x, p2.x ), std::min( p1.y, p2.y ), std::min( p1.z, p2.z ) },
      pMax{ std::max( p1.x, p2.x ), std::max( p1.y, p2.y ), std::max( p1.z, p2.z ) }
    {
    }

    const Point3< T >& operator[]( int i ) const
    {
        if ( i == 0 )
            return pMin;
        return pMax;
    }

    Point3< T >& operator[]( int i )
    {
        if ( i == 0 )
            return pMin;
        return pMax;
    }

    Point3< T > Corner( int corner ) const
    {
        return Point3< T >( ( *this )[ ( corner & 1 ) ].x, ( *this )[ ( corner & 2 ) ? 1 : 0 ].y,
                            ( *this )[ ( corner & 4 ) ? 1 : 0 ].z );
    }

    Vector3< T > Diagonal() const { return pMax - pMin; }

    T SurfaceArea() const
    {
        auto d = Diagonal();
        return 2 * ( d.x * d.y + d.x * d.z + d.y * d.z );
    }

    T Volume() const
    {
        auto d = Diagonal();
        return d.x * d.y * d.z;
    }

    int MaximumExtent() const
    {
        auto d = Diagonal();
        if ( d.x > d.y && d.x > d.z )
            return 0;
        else if ( d.y > d.z )
            return 1;
        return 2;
    }

    Point3< T > Lerp( const Point3f& t ) const
    {
        return Point3< T >(::Lerp( t.x, pMin.x, pMax.x ), ::Lerp( t.y, pMin.y, pMax.y ),
                           ::Lerp( t.z, pMin.z, pMax.z ) );
    }

    Vector3< T > Offset( const Point3< T >& p ) const
    {
        auto o = p - pMin;
        if ( pMax.x > pMin.x )
            o.x /= pMax.x - pMin.x;
        if ( pMax.y > pMin.y )
            o.y /= pMax.y - pMin.y;
        if ( pMax.z > pMin.z )
            o.z /= pMax.z - pMin.z;
        return o;
    }

    void BoundingSphere( Point3< T >* center, Float* radius ) const
    {
        *center = ( pMin + pMax ) / 2;
        *radius = Inside( *center, *this ) ? Distance( *center, pMax ) : 0;
    }
};

template < typename T > Bounds3< T > Union( const Bounds3< T >& b, const Point3< T >& p )
{
    return Bounds3< T >( Point3< T >( std::min( b.pmin.x, p.x ), std::min( b.pMin.y, p.y ),
                                      std::min( b.pMin.z, p.z ) ),
                         Point3< T >( std::max( b.pMax.x, p.x ), std::max( b.pMax.y, p.y ),
                                      std::max( b.pMax.z, p.z ) ) );
}

template < typename T > Bounds3< T > Union( const Bounds3< T >& b1, const Bounds3< T >& b2 )
{
    return Bounds3< T >(
      Point3< T >( std::min( b1.pmin.x, b2.pMin.x ), std::min( b1.pMin.y, b2.pMin.y ),
                   std::min( b1.pMin.z, b2.pMin.z ) ),
      Point3< T >( std::max( b1.pMax.x, b2.pMax.x ), std::max( b1.pMax.y, b2.pMax.y ),
                   std::max( b1.pMax.z, b2.pMax.z ) ) );
}

template < typename T > bool Overlaps( const Bounds3< T >& b1, const Bounds3< T >& b2 )
{
    bool x = ( b1.pMax.x >= b2.pMin.x ) && ( b1.pMin.x <= b2.pMax.x );
    bool y = ( b1.pMax.y >= b2.pMin.y ) && ( b1.pMin.y <= b2.pMax.y );
    bool z = ( b1.pMax.z >= b2.pMin.z ) && ( b1.pMin.z <= b2.pMax.z );
    return ( x && y && z );
}

template < typename T > bool Inside( const Point3< T >& p, const Bounds3< T >& b )
{
    return ( p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y &&
             p.z >= b.pMin.z && p.z <= b.pMax.z );
}

template < typename T > bool InsideExclusive( const Point3< T >& p, const Bounds3< T >& b )
{
    return ( p.x >= b.pMin.x && p.x < b.pMax.x && p.y >= b.pMin.y && p.y < b.pMax.y &&
             p.z >= b.pMin.z && p.z < b.pMax.z );
}

template < typename T, typename U > inline Bounds3< T > Expand( const Bounds3< T >& b, U delta )
{
    return Bounds3< T >( b.pMin - Vector3< T >( delta, delta, delta ),
                         b.pMax + Vector3< T >( delta, delta, delta ) );
}

typedef Bounds2< Float > Bounds2f;
typedef Bounds2< int > Bounds2i;
typedef Bounds3< Float > Bounds3f;
typedef Bounds3< int > Bounds3i;

class Bounds2iIterator : public std::forward_iterator_tag {
  public:
    Bounds2iIterator( const Bounds2i& b, const Point2i& pt ) : p{ pt }, bounds{ &b } {}
    Bounds2iIterator operator++()
    {
        advance();
        return *this;
    }
    Bounds2iIterator operator++( int )
    {
        Bounds2iIterator old = *this;
        advance();
        return old;
    }
    bool operator==( const Bounds2iIterator& bi ) const { return p == bi.p && bounds == bi.bounds; }
    bool operator!=( const Bounds2iIterator& bi ) const { return p != bi.p || bounds != bi.bounds; }

    Point2i operator*() const { return p; }

  private:
    Point2i p;
    const Bounds2i* bounds;

    void advance()
    {
        ++p.x;
        if ( p.x == bounds->pMax.x ) {
            p.x = bounds->pMin.x;
            ++p.y;
        }
    }
};

#endif /* geometry_h */
