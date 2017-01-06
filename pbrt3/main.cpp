//
//  main.cpp
//  pbrt3
//
//  Created by Ilya Rostovtsev on 1/3/17.
//  Copyright © 2017 Ilya Rostovtsev. All rights reserved.
//

#include "geometry.h"
#include <iostream>

template < typename T > void pv( const Vector2< T >& v )
{
    std::cout << v.x << ", " << v.y << '\n';
}

int main( int argc, const char* argv[] )
{
    Bounds3f test( Point3f( -10, -10, -10 ), Point3f( 10, 10, 10 ) );
    auto t = test.Lerp( Point3f( 0.53, .52, .51 ) );
    std::cout << t.x << ", " << t.y << ", " << t.z << '\n';
    return 0;
}
