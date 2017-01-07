//
//  sphere.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/7/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef sphere_hpp
#define sphere_hpp

#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Sphere : public Shape {
  public:
    Sphere( const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
            Float radius, Float zMin, Float zMax, Float phiMax );
    Bounds3f ObjectBound() const override;
    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;
    bool IntersectP( const Ray& r, bool testAlphaTexture ) const override;
    Float Area() const { return phiMax * radius * ( zMax - zMin ); }

  private:
    const Float radius;
    const Float zMin, zMax;
    const Float thetaMin, thetaMax, phiMax;
};

} /* namespace pbrt */
#endif /* sphere_hpp */
