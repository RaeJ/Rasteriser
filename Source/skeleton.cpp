#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "Bresenham.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::vec2;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

vec4 camera( 0, 0, -3.00, 1 );
int focal = SCREEN_WIDTH;
vector<Triangle> triangles;

vec3 theta( 0.0, 0.0, 0.0 );

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, glm::ivec2& p );
void TransformationMatrix(glm::mat4& m);
void Rotate();

int main( int argc, char* argv[] )
{
  LoadTestModel(triangles);

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.png" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  glm::mat4 matrix;  TransformationMatrix(matrix);

  for( uint32_t i=0; i<triangles.size(); ++i )
    {
      vector<ivec2> positions(3);

      vector<vec4> vertices(3);
      vertices[0] = triangles[i].v0;
      vertices[1] = triangles[i].v1;
      vertices[2] = triangles[i].v2;
      vec3 colour(1,1,1);
      
      for(int v=0; v<3; ++v)
        {
          vec4 vertex = matrix * vertices[v];

          ivec2 projected;

          VertexShader( vertex, projected );

          int x = projected.x; int y = projected.y;
          if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
            PutPixelSDL( screen, projected.x, projected.y, colour );
          }
          positions[v] = projected;
        }
        DrawLineBRS( screen, positions[0], positions[1], colour );
        DrawLineBRS( screen, positions[1], positions[2], colour );
        DrawLineBRS( screen, positions[2], positions[0], colour );
      }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

void VertexShader( const vec4& v, glm::ivec2& p ){
  int x = (int) ( focal * ( v.x / (float) v.z ) ) + ( SCREEN_WIDTH / (float) 2 );
  int y = (int) ( focal * ( v.y / (float) v.z ) ) + ( SCREEN_HEIGHT / (float) 2 );

  p.x = x;  p.y = y;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result ){
  int N = result.size();
  vec2 step = vec2(b-a) / float(max(N-1,1));
  vec2 current( a );
  for( int i=0; i<N; ++i )
  {
    result[i] = current;
    current += step;
  }
}

void TransformationMatrix(glm::mat4& M){
	  M[0][0] = cos(theta.y) * cos(theta.z);
    M[0][1] = cos(theta.y) * sin(theta.z);
    M[0][2] = -sin(theta.y);
    M[0][3] = 0;

    M[1][0] = (-cos(theta.x)*sin(theta.z)) + (sin(theta.x)*sin(theta.y)*cos(theta.z));
    M[1][1] = (cos(theta.x)*cos(theta.z)) + (sin(theta.x)*sin(theta.y)*sin(theta.z));
    M[1][2] = sin(theta.x)*cos(theta.y);
    M[1][3] = 0;

    M[2][0] = (sin(theta.x)*sin(theta.z)) + (cos(theta.x)*sin(theta.y)*cos(theta.z));
    M[2][1] = (-sin(theta.x)*cos(theta.z)) +(cos(theta.x)*sin(theta.y)*sin(theta.z));
    M[2][2] = cos(theta.x)*cos(theta.y);
    M[2][3] = 0;

    M[3][0] = -camera.x;
    M[3][1] = -camera.y;
    M[3][2] = -camera.z;
    M[3][3] = 1;

  }
