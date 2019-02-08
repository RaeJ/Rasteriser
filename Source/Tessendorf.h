#ifndef TESSENDORF_H
#define TESSENDORF_H

#include <glm/glm.hpp>
#include "TestModelH.h"

typedef struct
{
  std::vector<vec4> geometric_points;
} Grid;

Grid CreateSurface( int triangle_number, float height );

Grid CreateSurface( int triangle_number, float height  ){
  int width_points = triangle_number + 1;
  int total_points = pow( width_points, 2 );
  float triangle_size = ACTUAL_WIDTH / (float) triangle_number;

  std::vector<vec4> grid_points( total_points );
  for( int i=0; i<total_points; i++ ){
    grid_points[i].x = ( i % width_points ) * triangle_size;
    grid_points[i].z = floor( i / width_points ) * triangle_size;
    grid_points[i].y = height;

    grid_points[i]    -= SHIFT;
    grid_points[i].x  *= -1;
    grid_points[i].y  *= -1;
    grid_points[i].w   = 1.0;
  }
  Grid new_grid;
  new_grid.geometric_points = grid_points;

  return new_grid;
}

#endif
