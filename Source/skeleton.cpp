#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
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
float pi = PI;

vector<Triangle> triangles;

vec3 theta( 0.0, 0.0, 0.0 );
vec3 light_position(0,-0.5,-0.7);
vec3 light_power = 11.1f*vec3( 1, 1, 1 );
vec3 indirect_light = 0.5f*vec3( 1, 1, 1 );

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void VertexShader( const Vertex& v, Pixel& p );
void ComputePolygonRows( screen* screen, const vector<Vertex>& vertices );
void TransformationMatrix(glm::mat4& m);
void Rotate();
void UserInput();
void PixelShader( screen* screen, const Pixel& p );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
vec3 PixelIllumination( Pixel p );

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
  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ){
      depthBuffer[y][x] = 0;
    }
  }

  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  mat4 matrix;  TransformationMatrix(matrix);

  for( uint32_t i=0; i<triangles.size(); ++i )
    {
      vec4 normal = triangles[i].normal;
      vec3 colour = triangles[i].colour;
      vector<Vertex> vertices(3);

      vertices[0].position    = matrix * triangles[i].v0;
      vertices[0].normal      = normal;
      vertices[0].reflectance = colour;

      vertices[1].position    = matrix * triangles[i].v1;
      vertices[1].normal      = normal;
      vertices[1].reflectance = colour;

      vertices[2].position    = matrix * triangles[i].v2;
      vertices[2].normal      = normal;
      vertices[2].reflectance = colour;

      ComputePolygonRows( screen, vertices );
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

void ComputePolygonRows( screen* screen, const vector<Vertex>& vertices ){
  int V = vertices.size();
  vector<Pixel> projectedVertices( V );

  for( int i=0; i<V; i++ ){
    VertexShader( vertices[i], projectedVertices[i] );
  }

  vector< vector<Pixel> > edges( V );
  for( int i=0; i<V; i++ ){
    int j = ( i + 1 ) % V;
    Pixel p_i1 = projectedVertices[i];
    Pixel p_i2 = projectedVertices[j];

    int delta_x = abs( p_i1.x - p_i2.x );
    int delta_y = abs( p_i1.y - p_i2.y );
    int h = (int) sqrt( ( pow( delta_x, 2 ), pow( delta_y, 2 ) ) );
    int p_num = max( h, max( delta_x, delta_y ) ) + 1;

    vector<Pixel> result( p_num );

    Interpolate( p_i1, p_i2, result );

    edges[i] = result;
  }

  int y1 = +numeric_limits<int>::max(); int y2 = -numeric_limits<int>::max();
  for( int i=0; i<V; i++ ){
    Pixel projected = projectedVertices[i];

    if( projected.y < y1 ){ y1 = projected.y; }
    if( projected.y > y2 ){ y2 = projected.y; }
  }

  int ROWS = y2 - y1 + 1;

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

      int index = y - y1;

      if( edge[j].x < leftPixels[index].x ){
        leftPixels[index].x             = edge[j].x;
        leftPixels[index].y             = edge[j].y;
        leftPixels[index].zinv          = edge[j].zinv;
        leftPixels[index].reflectance   = edge[j].reflectance;
        leftPixels[index].pos4d         = edge[j].pos4d;
        leftPixels[index].normal        = edge[j].normal;
      }
      if( edge[j].x > rightPixels[index].x){
        rightPixels[index].x            = edge[j].x;
        rightPixels[index].y            = edge[j].y;
        rightPixels[index].zinv         = edge[j].zinv;
        rightPixels[index].reflectance  = edge[j].reflectance;
        rightPixels[index].pos4d        = edge[j].pos4d;
        rightPixels[index].normal       = edge[j].normal;
      }

    }
  }

  for( int i=0; i<ROWS; ++i )
  {
    Pixel left, right;
    left = leftPixels[i];
    right = rightPixels[i];

    int pixels = right.x - left.x + 1;
    vector<Pixel> line( pixels );

    Interpolate( left, right, line );

    for( int j=0; j<line.size(); j++ ){
      PixelShader( screen, line[j] );
    }
  }
}

void VertexShader( const Vertex& vertex, Pixel& p ){
  mat4 matrix;  TransformationMatrix(matrix);

  // 4D position
  vec4 v = vertex.position;
  p.pos4d = glm::inverse( matrix ) * v;

  // The 2D position
  int x = (int) ( focal * ( v.x / (float) v.z ) ) + ( SCREEN_WIDTH / (float) 2 );
  int y = (int) ( focal * ( v.y / (float) v.z ) ) + ( SCREEN_HEIGHT / (float) 2 );

  p.x = x;  p.y = y;

  // The depth of the point
  if( v.z != 0.0 ){
    p.zinv = ( float ) ( 1 / ( float ) v.z );
  } else {
    p.zinv = 0;
  }

  p.normal      = vertex.normal;
  p.reflectance = vertex.reflectance;
}

void PixelShader( screen* screen, const Pixel& p )
{
  int x = p.x;
  int y = p.y;
  vec3 illumination = PixelIllumination( p );

  if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
    if( p.zinv > depthBuffer[y][x] )
    {
      depthBuffer[y][x] = p.zinv;
      PutPixelSDL( screen, x, y, p.reflectance * ( illumination + indirect_light ) );
    }
  }
}

vec3 PixelIllumination( Pixel p )
{
  vec3 radius = glm::normalize( light_position - vec3( p.pos4d ) );
  vec3 normal = glm::normalize( vec3( p.normal ) );
  float diffuse = glm::dot( radius, normal );
  vec3 power = light_power * diffuse;
  if ( diffuse < 0  ){ power = vec3( 0, 0, 0 ); }
  float denominator = 4 * pi * glm::dot( radius, radius );

  return ( power / denominator );
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ){
  float x, y, zinv;
  vec4 pos4d;

  int N = result.size();
  vec3 colour = a.reflectance;
  vec4 normal = a.normal;


  if( N == 1 )
  {
    result[0].x           = (int) ( b.x + a.x ) / (float) 2;
    result[0].y           = (int) ( b.y + a.y ) / (float) 2;
    result[0].zinv        = ( b.zinv + a.zinv ) / (float) 2;
    result[0].pos4d       = ( b.pos4d + a.pos4d ) / (float) 2;
    result[0].reflectance = colour;
    result[0].normal      = normal;
  }
  else
  {
    x     = ( b.x - a.x ) / ( float ) ( N - 1 );
    y     = ( b.y - a.y ) / ( float ) ( N - 1 );
    zinv  = ( b.zinv - a.zinv ) / ( float ) ( N - 1 );
    pos4d = ( b.pos4d - a.pos4d ) / (float) ( N - 1 );

    for(int i=0; i<result.size(); i++){
      int result_x = (a.x + (i*x));
      int result_y = (a.y + (i*y));

      if( result_y > max( a.y, b.y ) ){
        result_y = max( a.y, b.y );
      }
      if( result_y < min( a.y, b.y ) ){
        result_y = min( a.y, b.y );
      }

      result[i].x           = result_x;
      result[i].y           = result_y;
      result[i].zinv        = a.zinv + ( i * zinv );
      result[i].pos4d       = a.pos4d + ( (float) i * pos4d );
      result[i].reflectance = colour;
      result[i].normal      = normal;

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
