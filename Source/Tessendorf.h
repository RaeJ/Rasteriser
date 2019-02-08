#ifndef TESSENDORF_H
#define TESSENDORF_H

#include <glm/glm.hpp>
#include "TestModelH.h"

typedef struct
{
  int side_points;
  std::vector<vec4> geometric_points;
} Grid;

Grid grid;

void CreateSurface( int triangle_number, float height );
void LoadGrid( std::vector<Triangle>& triangles );

void CreateSurface( int triangle_number, float height  ){
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
  new_grid.side_points = width_points;

  grid = new_grid;
}

void LoadGrid( std::vector<Triangle>& triangles )
{
  int width = grid.side_points;
  for( int col=0; col<width; col++ ){
    for( int row=0; row<width; row++ ){
      int i = ( row * width ) + col;
      if( ( i % width ) != ( width - 1 ) && ( row % width ) != ( width - 1 ) ){
          vec4 l      = grid.geometric_points[i];
          vec4 r      = grid.geometric_points[i + 1];
          vec4 down_l = grid.geometric_points[i + width];
          vec4 down_r = grid.geometric_points[i + width + 1];
          
          Triangle A  = Triangle( l, r, down_l, vec3(1.0,0,0) );
          Triangle B  = Triangle( down_l, down_r, r, vec3(0,1.0,0) );

          A.ComputeNormal(); B.ComputeNormal();

          triangles.push_back( A );
          triangles.push_back( B );
      }
    }
  }
}

#endif
