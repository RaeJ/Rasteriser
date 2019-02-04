#ifndef BRESENHAM_H
#define BRESENHAM_H

#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include <stdint.h>
#include <string>
#include <vector>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;

void DrawLineBRS( screen* screen, ivec2 a, ivec2 b, vector<ivec2>& result, vec3 colour );


void DrawLineBRS( screen* screen, ivec2 a, ivec2 b, vector<ivec2>& result, vec3 colour ){
  int x, y, xe, ye;
  int x1 = a.x;         int y1 = a.y;
  int x2 = b.x;         int y2 = b.y;
  int dx = x2 - x1;     int dy = y2 - y1;
  int dx1 = fabs(dx);   int dy1 = fabs(dy);
  int px = 2*dy1 - dx1; int py = 2*dx1 - dy1;

  int SCREEN_WIDTH = screen -> width;
  int SCREEN_HEIGHT = screen -> height;

  if( dy1 <= dx1 ){
    if( dx >= 0 ){
      x = x1;
      y = y1;
      xe = x2;
    } else {
      x = x2;
      y = y2;
      xe = x1;
    }
    if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
      PutPixelSDL( screen, x, y, colour );
    }
    for( int i=0; x<xe; i++ ){
      x = x+1;
      if( px < 0 ){
        px = px + 2*dy1;
      } else {
        if( ( dx<0 && dy<0 ) || ( dx>0 && dy>0 ) ){
          y = y+1;
        } else {
          y = y-1;
        }
        px = px + 2*( dy1 - dx1 );
      }
      if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
        PutPixelSDL( screen, x, y, colour );
      }
      ivec2 point; point.x = x; point.y = y;
      result[i] = (point);
    }
  } else {
    if ( dy >= 0 ){
      x = x1;
      y = y1;
      ye = y2;
    } else {
      x = x2;
      y = y2;
      ye = y1;
    }
    if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
      PutPixelSDL( screen, x, y, colour );
    }
    for( int i=0; y<ye; i++ ){
      y = y+1;
      if( py <= 0 ){
        py = py + 2*dx1;
      } else {
        if( ( dx<0 && dy<0 ) || ( dx>0 && dy>0 ) ){
          x = x+1;
        } else {
          x = x-1;
        }
        py = py + 2*( dx1 - dy1 );
      }
      if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
        PutPixelSDL( screen, x, y, colour );
      }
      ivec2 point; point.x = x; point.y = y;
      result[i] = (point);
    }
  }
}

#endif
