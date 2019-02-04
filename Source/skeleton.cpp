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
#define SCREEN_HEIGHT 320
#define FULLSCREEN_MODE false
#define PI 3.14159265

vec4 camera( 0, 0, -3.00, 1 );
int focal = SCREEN_WIDTH;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

vector<Triangle> triangles;

vec3 theta( 0.0, 0.0, 0.0 );

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, Pixel& p );
void ComputePolygonRows( screen* screen, const vector<vec4>& vertices, vec3& c );
void TransformationMatrix(glm::mat4& m);
void Rotate();
void UserInput();
void PixelShader( screen* screen, const Pixel& p, vec3& c );

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
  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ){
      depthBuffer[y][x] = 0;
    }
  }

  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  mat4 matrix;  TransformationMatrix(matrix);

  for( uint32_t i=0; i<triangles.size(); ++i )
    {
      vector<vec4> vertices(3);
      vertices[0] = matrix * triangles[i].v0;
      vertices[1] = matrix * triangles[i].v1;
      vertices[2] = matrix * triangles[i].v2;

      vec3 colour = triangles[i].colour;

      ComputePolygonRows( screen, vertices, colour );
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
  // std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/

  UserInput();
}

void ComputePolygonRows( screen* screen, const vector<vec4>& vertices, vec3& colour ){

  int V = vertices.size();

  vector< vector<Pixel> > edges( V );

  vector<Pixel> projectedVertices( V );
  for( int i=0; i<V; i++ ){
    VertexShader( vertices[i], projectedVertices[i] );
  }

  for( int i=0; i<V; i++ ){
    int j = (i+1)%V;
    Pixel current = projectedVertices[i];
    Pixel next = projectedVertices[j];
    cout << "z1: " << current.zinv << "\n";
    cout << "z2: " << next.zinv << "\n";

    int delta_x = abs(current.x - next.x);
    int delta_y = abs(current.y - next.y);

    vector<Pixel> result( max( delta_x, delta_y ) );

    DrawLineBRS( screen, current, next, result );

    edges[i] = result;
  }

  int y1 = +numeric_limits<int>::max(); int y2 = -numeric_limits<int>::max();
  for( int i=0; i<V; i++ ){
    Pixel projected = projectedVertices[i];
    int j = (i+1)%V;

    if( projected.y < y1 ){
      y1 = projected.y;
    }
    if( projected.y > y2 ){
      y2 = projected.y;
    }
  }

  int ROWS = y2 - y1 + 1;
  int offset = min( y1, y2 );

  vector<Pixel> leftPixels( ROWS );
  vector<Pixel> rightPixels( ROWS );

  for( int i=0; i<ROWS; ++i )
  {
    leftPixels[i].x  = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }

  for( int i=0; i<V; i++ ){

    vector<Pixel> edge = edges[i];
    for( int j=0; j<edge.size(); j++ ){
      int y = edge[j].y;
      int index = y-offset;

      if( edge[j].x < leftPixels[index].x ){
        leftPixels[index].x = edge[j].x;
        leftPixels[index].y = edge[j].y;
        leftPixels[index].zinv = edge[j].zinv;
      }
      if( edge[j].x > rightPixels[index].x){
        rightPixels[index].x = edge[j].x;
        rightPixels[index].y = edge[j].y;
        rightPixels[index].zinv = edge[j].zinv;
      }
    }
  }

  for( int i=0; i<ROWS; ++i )
  {
    Pixel left, right;
    left = leftPixels[i];
    right = rightPixels[i];

    vector<Pixel> result( abs( right.x - left.x ) );

    DrawLineBRS( screen, left, right, result );

    for( int j=0; j<result.size(); j++ ){
      PixelShader( screen, result[j], colour);
    }
  }
}

void VertexShader( const vec4& v, Pixel& p ){
  int x = (int) ( focal * ( v.x / (float) v.z ) ) + ( SCREEN_WIDTH / (float) 2 );
  int y = (int) ( focal * ( v.y / (float) v.z ) ) + ( SCREEN_HEIGHT / (float) 2 );

  if( v.z != 0.0 ){
    p.zinv = ( float ) ( 1 / ( float ) v.z );
  } else {
    p.zinv = 0;
  }

  p.d = v; 

  p.x = x;  p.y = y;
}

void PixelShader( screen* screen, const Pixel& p, vec3& c )
{
  int x = p.x;
  int y = p.y;
  cout << "pzinv: " << p.zinv << "\n";
  cout << "depth: " << depthBuffer[y][x] << "\n";
  if( p.zinv > depthBuffer[y][x] )
  {
    if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
      depthBuffer[y][x] = p.zinv;
      PutPixelSDL( screen, x, y, c );
    }
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

  void UserInput(){
    const uint8_t* keystate = SDL_GetKeyboardState( 0 );

    if( keystate == NULL ) {
      printf("keystate = NULL!\n");
    }
    // Rotation
    if( keystate[SDL_SCANCODE_UP] ) {
      theta.x -= PI/180;
    }
    if( keystate[SDL_SCANCODE_DOWN] ) {
      theta.x += PI/180;
    }
    if( keystate[SDL_SCANCODE_LEFT] ) {
      theta.y += PI/180;
    }
    if( keystate[SDL_SCANCODE_RIGHT] ) {
      theta.y -= PI/180;
    }
    // Move forward/back/left/right
    if( keystate[SDL_SCANCODE_W] ) {
      camera.z += 0.005;
    }
    if( keystate[SDL_SCANCODE_S] ) {
      camera.z -= 0.005;
    }
    if( keystate[SDL_SCANCODE_A] ) {
      camera.x -= 0.005;
    }
    if( keystate[SDL_SCANCODE_D] ) {
      camera.x += 0.005;
    }
    // Reset state
    if( keystate[SDL_SCANCODE_R] ) {
      camera = vec4( 0, 0, -3.00, 1 );
      theta = vec3( 0.0, 0.0, 0.0 );
    }
  }
