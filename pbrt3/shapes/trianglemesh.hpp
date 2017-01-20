//
//  trianglemesh.hpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/19/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#ifndef trianglemesh_hpp
#define trianglemesh_hpp

#include "pbrt.hpp"
#include "shape.hpp"
#include "transform.hpp"

namespace pbrt {

struct TriangleMesh
{
    const int nTriangles, nVertices;
    std::vector< int > vertexIndices;
    std::unique_ptr< Point3f[] > p;
    std::unique_ptr< Normal3f[] > n;
    std::unique_ptr< Vector3f[] > s;
    std::unique_ptr< Point2f[] > uv;
    std::shared_ptr< Texture< Float > > alphaMask;

    TriangleMesh( const Transform& ObjectToWorld, int nTriangles, const int* vertexIndices,
                  int nVertices, const Point3f* P, const Vector3f* S, const Normal3f* N,
                  const Point2f* UV, const std::shared_ptr< Texture< Float > >& alphaMask );
};

class Triangle : public Shape {
  public:
    Triangle( const Transform* ObjectToWorld, const Transform* WorldToObject,
              bool reverseOrientation, const std::shared_ptr< TriangleMesh >& mesh, int triNumber );

    Bounds3f ObjectBound() const override;
    Bounds3f WorldBound() const;
    bool Intersect( const Ray& ray, Float* tHit, SurfaceInteraction* isect,
                    bool testAlphaTexture ) const override;
    bool IntersectP( const Ray& ray, bool testAlphaTexture = true ) const override;

    std::vector< std::shared_ptr< Shape > >
    CreateTriangleMesh( const Transform* ObjectToWorld, const Transform* WorldToObject,
                        bool reverseOrientation, int nTriangles, const int& vertexIndices,
                        int nVertices, const Point3f* p, const Vector3f* s, const Normal3f* n,
                        const Point2f* uv, const std::shared_ptr< Texture< Float > >& alphaMask );

    void GetUVs( Point2f uv[ 3 ] ) const;

  private:
    std::shared_ptr< TriangleMesh > mesh;
    const int* v; // vertex index
};

} /* namespace pbrt */
#endif /* trianglemesh_hpp */
