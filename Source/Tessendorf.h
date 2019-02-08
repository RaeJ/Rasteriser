#ifndef TESSENDORF_H
#define TESSENDORF_H

#include <glm/glm.hpp>
#include <vector>
#include "TestModelH.h"

typedef struct
{
  int width_n;
  int depth_n;
  std::vector<vec4> geometric_points;
  float height;
} Grid;

Grid CreateSurface( float width, float depth, float height, float triangle_width );

Grid CreateSurface( float width, float depth, float height, float triangle_width ){
  Grid surface;
  // if( ( std::fmod( width, triangle_width ) ) != 0 ){
  //   std::cout << "Warning: Triangles will not cover the length of the specified width\n";
  // }
  // if( ( std::fmod( depth, triangle_width ) ) != 0 ){
  //   std::cout << "Warning: Triangles will not cover the length of the specified depth\n";
  // }
  surface.width_n   = floor( width / triangle_width );
  surface.depth_n   = floor( depth / triangle_width );
  surface.height    = height;

  int points_width = surface.width_n + 1;
  int points_depth = surface.depth_n + 1;
  int points_num = points_width * points_depth;

  std::vector<vec4> surface_points( points_num );

  for( int i=0; i<points_num; i++ ){
    int j = floor( i / points_width );
    surface_points[i].x = ( i % points_width ) * triangle_width;
    surface_points[i].z = j * triangle_width;
    surface_points[i].y = height;
    surface_points[i].w = 1.0;
  }

  return surface;
}

#endif
