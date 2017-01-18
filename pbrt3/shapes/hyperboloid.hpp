//
//  hyperboloid.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef hyperboloid_hpp
#define hyperboloid_hpp

#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Hyperboloid : public Shape {
  public:
    Hyperboloid( const Transform* ObjectToWorld, const Transform* WorldToObject,
                 bool reverseOrientation, const Point3f& p1, const Point3f& p2, Float tm );

    Bounds3f ObjectBound() const override;

    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;

    bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const override;

#define SQR( a ) ( ( a ) * ( a ) )
#define QUAD( a ) ( ( SQR( a ) ) * ( SQR( a ) ) )
    Float Area() const
    {
        return phiMax / 6.f *
               ( 2 * QUAD( p1.x ) - 2 * p1.x * p1.x * p1.x * p2.x + 2 * QUAD( p2.x ) +
                 2 * ( p1.y * p1.y + p1.y * p2.y + p2.y * p2.y ) *
                   ( SQR( p1.y - p2.y ) + SQR( p1.z - p2.z ) ) +
                 p2.x * p2.x * ( 5 * p1.y * p1.y + 2 * p1.y * p2.y - 4 * p2.y * p2.y +
                                 2 * SQR( p1.z - p2.z ) ) +
                 p1.x * p1.x * ( -4 * p1.y * p1.y + 2 * p1.y * p2.y + 5 * p2.y * p2.y +
                                 2 * SQR( p1.z - p2.z ) ) -
                 2 * p1.x * p2.x * ( p2.x * p2.x - p1.y * p1.y + 5 * p1.y * p2.y - p2.y * p2.y -
                                     p1.z * p1.z + 2 * p1.z * p2.z - p2.z * p2.z ) );
    }
#undef SQR
#undef QUAD

  private:
    Point3f p1, p2;
    Float zMin, zMax, phiMax, rMax, ah, ch;
};

} /* namespace pbrt */
#endif /* hyperboloid_hpp */
