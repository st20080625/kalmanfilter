#ifndef D_CPP_DRAW_HPP
#define D_CPP_DRAW_HPP
#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include <iostream>
#include "3d.hpp"
#include <vector>

using namespace cpp3d;
namespace draw{
vec2d Calc_Coords(const vec3d &a, vec2d &center);

void Draw_Line(SDL_Renderer *renderer, vec2d start, vec2d end, vec3d color);

void Draw_Circle(SDL_Renderer *renderer, vec2d center, int radius, vec3d color);

vector<vec3d> Calc_Circle(vec3d center, int radius);

void Draw_Triangle(SDL_Renderer *renderer, vec2d window_center, vec3d  point1, vec3d point2, vec3d point3, vec3d color);

void Draw_Cube(SDL_Renderer *renderer, vec2d window_center, vec3d point, int width, int height, int depth, float anglex, float angley, float anglez, vec3d color);

vector<vec3d> Calc_Ball(SDL_Renderer *renderer, vec3d ball_center, int radius);

void Draw_Ball(SDL_Renderer *renderer, vec2d window_center, vector<vec3d> points, float anglex, float angley, float anglez, vec3d color);
}
#endif