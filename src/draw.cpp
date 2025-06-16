#include <iostream>
#include <vector>
#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include "3d.hpp"
#include "draw.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

namespace draw
{
    using std::cout;
    using std::endl;
    using std::vector;
    using namespace cpp3d;

    vec2d Calc_Coords(const vec3d &a, vec2d &window_center)
    {
        float scale = 500 / (500 + a.z);
        float x = (a.x / scale);
        float y = (-a.y / scale);
        x += window_center.x;
        y += window_center.y;
        return vec2d(x, y);
    }

    void Draw_Line(SDL_Renderer *renderer, vec2d start, vec2d end, vec3d color)
    {
        SDL_SetRenderDrawColor(renderer, color.x, color.y, color.z, 255);
        SDL_RenderLine(renderer, start.x, start.y, end.x, end.y);
    }

    void Draw_Circle(SDL_Renderer *renderer, vec2d center, int radius, vec3d color)
    {
        int offsetX = radius;
        int offsetY = 0;
        int dx = 1;
        int dy = 1;
        int err = dx - (radius << 1);
        float centerX = center.x;
        float centerY = center.y;
        SDL_SetRenderDrawColor(renderer, color.x, color.y, color.z, 255);
        while (offsetX >= offsetY)
        {
            // 円の8方向に対称な点を描画
            SDL_RenderPoint(renderer, centerX + offsetX, centerY + offsetY);
            SDL_RenderPoint(renderer, centerX + offsetY, centerY + offsetX);
            SDL_RenderPoint(renderer, centerX - offsetY, centerY + offsetX);
            SDL_RenderPoint(renderer, centerX - offsetX, centerY + offsetY);
            SDL_RenderPoint(renderer, centerX - offsetX, centerY - offsetY);
            SDL_RenderPoint(renderer, centerX - offsetY, centerY - offsetX);
            SDL_RenderPoint(renderer, centerX + offsetY, centerY - offsetX);
            SDL_RenderPoint(renderer, centerX + offsetX, centerY - offsetY);

            if (err <= 0)
            {
                offsetY++;
                err += dy;
                dy += 2;
            }

            if (err > 0)
            {
                offsetX--;
                dx += 2;
                err += dx - (radius << 1);
            }
        }
    }

    vector<vec3d> Calc_Circle(vec3d center, int radius)
    {
        vector<vec3d> vectors;
        int offsetX = radius;
        int offsetY = 0;
        int dx = 1;
        int dy = 1;
        int err = dx - (radius << 1);
        float centerX = center.x;
        float centerY = center.y;
        while (offsetX >= offsetY)
        {
            vec3d point1 = vec3d(centerX + offsetX, centerY + offsetY, 0);
            vec3d point2 = vec3d(centerX + offsetY, centerY + offsetX, 0);
            vec3d point3 = vec3d(centerX - offsetY, centerY + offsetX, 0);
            vec3d point4 = vec3d(centerX - offsetX, centerY + offsetY, 0);
            vec3d point5 = vec3d(centerX - offsetX, centerY - offsetY, 0);
            vec3d point6 = vec3d(centerX - offsetY, centerY - offsetX, 0);
            vec3d point7 = vec3d(centerX + offsetY, centerY - offsetX, 0);
            vec3d point8 = vec3d(centerX + offsetX, centerY - offsetY, 0);

            vectors.push_back(point1);
            vectors.push_back(point2);
            vectors.push_back(point3);
            vectors.push_back(point4);
            vectors.push_back(point5);
            vectors.push_back(point6);
            vectors.push_back(point7);
            vectors.push_back(point8);
            if (err <= 0)
            {
                offsetY++;
                err += dy;
                dy += 2;
            }

            if (err > 0)
            {
                offsetX--;
                dx += 2;
                err += dx - (radius << 1);
            }
        }
        return vectors;
    }

    void Draw_Triangle(SDL_Renderer *renderer, vec2d window_center, vec3d point1, vec3d point2, vec3d point3, vec3d color)
    {
        vec2d center1 = Calc_Coords(point1, window_center);
        vec2d center2 = Calc_Coords(point2, window_center);
        vec2d center3 = Calc_Coords(point3, window_center);

        SDL_SetRenderDrawColor(renderer, color.x, color.y, color.z, 255);
        SDL_RenderLine(renderer, center1.x, center1.y, center2.x, center2.y);
        SDL_RenderLine(renderer, center2.x, center2.y, center3.x, center3.y);
        SDL_RenderLine(renderer, center3.x, center3.y, center1.x, center1.y);
    }

    void Draw_Cube(SDL_Renderer *renderer, vec2d window_center, vec3d point, int width, int height, int depth, float anglex, float angley, float anglez, vec3d color)
    {
        vec3d point1 = point;
        vec3d point2 = point + vec3d(width, 0, 0);
        vec3d point3 = point + vec3d(0, height, 0);
        vec3d point4 = point + vec3d(0, 0, depth);
        vec3d point5 = point + vec3d(width, height, 0);
        vec3d point6 = point + vec3d(width, height, depth);
        vec3d point7 = point + vec3d(width, 0, depth);
        vec3d point8 = point + vec3d(0, height, depth);

        vector<vec3d> points = {point1, point2, point3, point4, point5, point6, point7, point8};
        vector<vec3d> rotated_points;
        for (size_t i = 0; i < points.size(); i++)
        {
            vec3d rotated_point = rotate_with_quaternion_with_three_angles(points[i], anglex, angley, anglez);
            rotated_points.push_back(rotated_point);
        }
        vector<vec2d> coords_2d;
        for (size_t i = 0; i < rotated_points.size(); i++)
        {
            vec2d coords = Calc_Coords(rotated_points[i], window_center);
            coords_2d.push_back(coords);
        }
        Draw_Line(renderer, coords_2d[0], coords_2d[1], color);
        Draw_Line(renderer, coords_2d[0], coords_2d[2], color);
        Draw_Line(renderer, coords_2d[0], coords_2d[3], color);
        Draw_Line(renderer, coords_2d[1], coords_2d[4], color);
        Draw_Line(renderer, coords_2d[1], coords_2d[6], color);
        Draw_Line(renderer, coords_2d[2], coords_2d[4], color);
        Draw_Line(renderer, coords_2d[2], coords_2d[7], color);
        Draw_Line(renderer, coords_2d[3], coords_2d[7], color);
        Draw_Line(renderer, coords_2d[3], coords_2d[6], color);
        Draw_Line(renderer, coords_2d[4], coords_2d[5], color);
        Draw_Line(renderer, coords_2d[5], coords_2d[6], color);
        Draw_Line(renderer, coords_2d[5], coords_2d[7], color);
    }

    vector<vec3d> Calc_Ball(SDL_Renderer *renderer, vec3d ball_center, int radius)
    {
        // 球の分割数（緯度と経度の分割数を調整可能）
        const int lat_segments = 18; // 緯度方向の分割数
        const int lon_segments = 36; // 経度方向の分割数

        vector<vec3d> points;
        // 緯度方向のループ
        for (int lat = 0; lat <= lat_segments; ++lat)
        {
            float theta = lat * M_PI / lat_segments; // 緯度角（0からπまで）
            float sin_theta = sin(theta);
            float cos_theta = cos(theta);

            // 経度方向のループ
            for (int lon = 0; lon <= lon_segments; ++lon)
            {
                float phi = lon * 2.0f * M_PI / lon_segments; // 経度角（0から2πまで）
                float sin_phi = sin(phi);
                float cos_phi = cos(phi);

                // 球面上の点を計算
                vec3d point(
                    ball_center.x + radius * sin_theta * cos_phi,
                    ball_center.y + radius * sin_theta * sin_phi,
                    ball_center.z + radius * cos_theta);

                points.push_back(point);
            }
        }
        return points;
    }

    void Draw_Ball(SDL_Renderer *renderer, vec2d window_center, vector<vec3d> points, float anglex, float angley, float anglez, vec3d color)
    {
        
        SDL_SetRenderDrawColor(renderer, color.x, color.y, color.z, 255);
        for (size_t i = 0; i < points.size(); i++)
        {
            vec3d rotated_point = rotate_with_quaternion_with_three_angles(points[i], anglex, angley, anglez);
            vec2d coords = Calc_Coords(rotated_point, window_center);
            SDL_RenderPoint(renderer, coords.x, coords.y);
        }
    }
}