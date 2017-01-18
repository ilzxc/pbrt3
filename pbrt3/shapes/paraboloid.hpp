//
//  paraboloid.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef paraboloid_hpp
#define paraboloid_hpp

#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Paraboloid : public Shape {
  public:
    Paraboloid( const Transform* ObjectToWorld, const Transform* WorldToObject,
                bool reverseOrientation, Float radius, Float z1, Float z2, Float phiMax );

    Bounds3f ObjectBound() const override;

    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;

    bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const override;

    Float Area() const
    {
        return radius * std::sqrt( height * height + radius * radius ) * phiMax / 2;
    }

  private:
    const Float radius, zMin, zMax, phiMax;
};

} /* namespace pbrt */

#endif /* paraboloid_hpp */
