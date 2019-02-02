#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

vec4 C( 0, 0, -3.00,1 );
int focal = SCREEN_WIDTH;
vector<Triangle> triangles;

float theta = 0;
float yaw = 0;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, glm::ivec2& p );
void TransformationMatrix(glm::mat4x4 M);
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

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  glm::mat4x4 M;  TransformationMatrix(M);

  for( uint32_t i=0; i<triangles.size(); ++i )
    {
      vector<vec4> vertices(3);
      vertices[0] = triangles[i].v0;
      vertices[1] = triangles[i].v1;
      vertices[2] = triangles[i].v2;
      for(int v=0; v<3; ++v)
        {
          vec4 vertex = M * vertices[v];

          glm::ivec2 projPos;
          VertexShader( vertex, projPos );
          vec3 color(1,1,1);
          PutPixelSDL( screen, projPos.x, projPos.y, color );
        }
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
  int x = round(focal * ( v.x / (float) v.z ) + ( SCREEN_WIDTH / (float) 2 ));
  int y = round(focal * ( v.y / (float) v.z ) + ( SCREEN_HEIGHT / (float) 2 ));

  p.x = x;  p.y = y;
}

void TransformationMatrix(glm::mat4x4 M){
	  M[0][0] = cos( yaw );   M[1][0] = sin( yaw ) * sin( theta );  M[2][0] = sin( yaw ) * cos( theta );  M[3][0] = -C.x;
	  M[0][1] = 0;            M[1][1] = cos( theta );               M[2][1] = -sin( theta );              M[3][1] = -C.y;
	  M[0][2] = -sin( yaw );  M[1][2] = cos( yaw ) * sin( theta );  M[2][2] = cos( yaw ) * cos( theta );  M[3][2] = -C.z;
    M[0][3] = 0;            M[1][3] = 0;                          M[2][3] = 0;                          M[3][3] = 1;
	}
