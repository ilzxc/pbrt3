//
//  disk.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/17/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef disk_hpp
#define disk_hpp

#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Disk : public Shape {
  public:
    Disk( const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation,
          Float height, Float radius, Float innerRadius, Float phiMax );

    Bounds3f ObjectBound() const override;

    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;

    bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const override;

    Float Area() const
    {

        return phiMax * Float{ 0.5f } * ( radius * radius - innerRadius * innerRadius );
    }

  private:
    const Float height, radius, innerRadius, phiMax;
};

} /* namespace pbrt */
#endif /* disk_hpp */
