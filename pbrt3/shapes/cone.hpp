//
//  cone.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef cone_hpp
#define cone_hpp

#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Cone : public Shape {
  public:
    Cone( const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
          Float height, Float radius, Float phiMax );

    Bounds3f ObjectBound() const override;

    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;

    bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const override;

    Float Area() const
    {
        return radius * std::sqrt( height * height + radius * radius ) * phiMax / 2;
    }

  private:
    const Float height, radius, phiMax;
};

} /* namespace pbrt */
#endif /* cone_hpp */
