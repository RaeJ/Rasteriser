#ifndef TESSENDORF_H
#define TESSENDORF_H

#include <glm/glm.hpp>
#include "TestModelH.h"
#include <complex>

typedef struct
{
  float height;
  int side_points;
  std::vector<vec4> geometric_points;
} Grid;

Grid GRID;

// Water Stuff
float gravity = 9.8;
glm::vec2 wind_dir( 0.316, 0.316 );
float max_wave = 0.2;
float amplitude = HALF_W/5;
// float max_wave = ( glm::dot( wind_dir, wind_dir ) ) / gravity;
float step = HALF_W/5;
// int N = 64;
// int M = 64;

double e_r = 0.5;
double e_i = 0.5;

void CreateSurface( int triangle_number, float height );
void LoadGrid( std::vector<Triangle>& triangles );
void UpdateHeight( double time );

void CreateSurface( int triangle_number, float height  )
{
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
  new_grid.height = height;

  GRID = new_grid;
}

void LoadGrid( std::vector<Triangle>& triangles )
{
  int width = GRID.side_points;
  for( int col=0; col<width; col++ ){
    for( int row=0; row<width; row++ ){
      int i = ( row * width ) + col;
      if( ( i % width ) != ( width - 1 ) && ( row % width ) != ( width - 1 ) ){
          vec4 l      = GRID.geometric_points[i];
          vec4 r      = GRID.geometric_points[i + 1];
          vec4 down_l = GRID.geometric_points[i + width];
          vec4 down_r = GRID.geometric_points[i + width + 1];

          Triangle A  = Triangle( l, r, down_l, vec3(0.25,0.52,0.95) );
          A.ComputeNormal();  triangles.push_back( A );

          Triangle B  = Triangle( down_l, down_r, r, vec3(0.25,0.34,0.96) );
          B.ComputeNormal();  triangles.push_back( B );
      }
    }
  }
}

void UpdateHeight( double time )
{
  // if( fmod( time, 2.0 ) == 0 ){
  //   e_r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  // }

  float multiplier = ( 1 / sqrt( 2 ) ) ;

  for( int i = 0; i<GRID.geometric_points.size(); i++ ){
    glm::vec2 x_0( GRID.geometric_points[i].x, GRID.geometric_points[i].z );
    std::complex<float> result = 0;

    for( float n = 0; n < HALF_W; n = n + step ){
      for( float m = 0; m < HALF_W; m = m + step ){
        float k_x = ( 2 * PI * ( n - HALF_W/2 ) ) / max_wave;
        float k_z = ( 2 * PI * ( m - HALF_W/2 ) ) / max_wave;

        glm::vec2 k( k_x, k_z );
        float nk = glm::length( k );
        float w = sqrt( gravity * nk );

        float k4 = pow( nk, 4 );

        std::complex<float> eikx0( cos( glm::dot( k, x_0 ) ), sin( glm::dot( k, x_0 ) ) );

        std::complex<float> neg_exp( cos( -time * w ), sin( -time * w ) );
        std::complex<float> pos_exp( cos( time * w ), sin( time * w ) );

        float phil_pos = 0; float phil_neg = 0;

        if( k4 != 0 ){
          phil_pos = amplitude * ( exp( -1 / ( glm::dot( k, k ) ) ) / k4 )
                               * pow( abs( glm::dot( k, wind_dir ) ), 2 );
          phil_neg = amplitude * ( exp( -1 / ( glm::dot( k, k ) ) ) / k4 )
                               * pow( abs( glm::dot( -k, wind_dir ) ), 2 );
        }


        std::complex<float> sqr_p( sqrt( phil_pos ) );
        std::complex<float> sqr_n( sqrt( phil_neg ) );

        std::complex<float> random_vars( e_r, e_i );

        std::complex<float> part_a = multiplier * random_vars * sqr_p;
        std::complex<float> part_b = multiplier * random_vars * sqr_n;
        std::complex<float> partial = ( ( part_a * pos_exp ) + ( part_b * neg_exp ) );

        result +=  partial * eikx0;
      }
    }
    GRID.geometric_points[i].y = abs( result );
    GRID.geometric_points[i].y += GRID.height;
    GRID.geometric_points[i].y *= -1;
  }
}

#endif
