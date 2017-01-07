//
//  shape.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/7/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "shape.hpp"
#include "transform.hpp"

namespace pbrt {

Shape::Shape( const Transform* objectToWorld, const Transform* worldToObject,
              bool reverseOrientation )
: ObjectToWorld{ objectToWorld },
  WorldToObject{ worldToObject },
  reverseOrientation{ reverseOrientation },
  transformSwapsHandidness{ ObjectToWorld->SwapsHandedness() }
{
}

Bounds3f Shape::WorldBound() const { return ( *ObjectToWorld )( ObjectBound() ); }

} /* namespace pbrt */
