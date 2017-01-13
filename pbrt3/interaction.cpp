//
//  interaction.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/12/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "interaction.hpp"
#include "shape.hpp"

namespace pbrt {

Interaction::Interaction( const Point3f& p, const Normal3f& n, const Vector3f& pError,
                          const Vector3f& wo, Float time, MediumInterface* mediumInterface )
: p{ p }, time{ time }, pError{ pError }, wo{ wo }, n{ n }, mediumInterface{ mediumInterface }
{
}

SurfaceInteraction::SurfaceInteraction( const Point3f& p, const Vector3f& pError, const Point2f& uv,
                                        const Vector3f& wo, const Vector3f& dpdu,
                                        const Vector3f& dpdv, const Normal3f& dndu,
                                        const Normal3f& dndv, Float time, const Shape* shape )
: Interaction{ p, Normal3f( Normalize( Cross( dpdu, dpdv ) ) ), pError, wo, time, nullptr },
  uv{ uv },
  dpdu{ dpdu },
  dpdv{ dpdv },
  dndv{ dndv },
  shape{ shape }
{
    shading.n = n;
    shading.dpdu = dpdu;
    shading.dpdv = dpdv;
    shading.dndu = dndu;
    shading.dndv = dndv;

    if ( shape && shape->reverseOrientation ^ shape->transformSwapsHandedness ) {
        n *= -1;
        shading.n *= -1;
    }
}

void SurfaceInteraction::SetShadingGeometry( const Vector3f& dpdus, const Vector3f& dpdvs,
                                             const Normal3f& dndus, const Normal3f& dndvs,
                                             bool orientationisAuthoritative )
{
    shading.n = Normalize( static_cast< Normal3f >( Cross( dpdus, dpdvs ) ) );
    if ( shape && ( shape->reverseOrientation ^ shape->transformSwapsHandedness ) )
        shading.n = -shading.n;
    if ( orientationisAuthoritative )
        n = FaceForward( n, shading.n );
    else
        shading.n = FaceForward( shading.n, n );

    shading.dpdu = dpdus;
    shading.dpdv = dpdvs;
    shading.dndu = dndus;
    shading.dndv = dndvs;
}
}
