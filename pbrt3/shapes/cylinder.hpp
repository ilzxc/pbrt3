//
//  cylinder.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/14/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef cylinder_hpp
#define cylinder_hpp

#include "geometry.hpp"
#include "pbrt.hpp"
#include "shape.hpp"

namespace pbrt {

class Cylinder : public Shape {
  public:
    Cylinder( const Transform* o2w, const Transform* w20, bool ro, Float rad, Float z0, Float z1,
              Float pm );

    Bounds3f ObjectBound() const override;
    bool Intersect( const Ray& r, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;
    bool IntersectP( const Ray& r, bool testAlphaTexture ) const override;

    inline Float Area() const { return ( zmax - zmin ) * phiMax * radius; }

  private:
    Float radius, zMin, zMax, phiMax;
};

} /* namespace pbrt */
#endif /* cylinder_hpp */
