#ifndef CREATE_ROAD_HPP
#define CREATE_ROAD_HPP

#include "road_types.hpp"
#include <raylib.h>
#include <string>
#include <vector>



void check_grid_collisions(std::vector<Square>& grid)
{
    Vector2 mouse_pos = GetMousePosition();
    for (int i = 0; i < grid.size(); i++)
    {
        Square& square = grid[i];
        square.is_hovering =
               mouse_pos.x > square.x && mouse_pos.x < square.x + square.size
            && mouse_pos.y > square.y && mouse_pos.y < square.y + square.size;

        if (square.is_hovering) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                square.is_selected = true;
            } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                square.is_selected = false;
            }
        }
    }
}


RoadLine create_line(Vector2 mouse_pos)
{
    auto point = Point{.x=(int)mouse_pos.x, .y=(int)mouse_pos.y, .is_hovering=false};
    auto line = RoadLine{.points={point}};
    return line;
}


void append_to_line(Vector2 mouse_pos, RoadLine* line)
{
    auto point = Point{.x=(int)mouse_pos.x, .y=(int)mouse_pos.y, .is_hovering=false};
    line->points.push_back(point);
}


void draw(const std::vector<Square>& grid, const std::vector<RoadLine> lines, Mode current_mode)
{
    BeginDrawing();
    ClearBackground(RAYWHITE);

    std::string mode_string = "";
    switch (current_mode) {
        case MODE_DEFAULT:
            mode_string = "default";
            break;
        case MODE_LINE:
            mode_string = "line";
            break;
    }

    for (auto square: grid)
    {
        DrawRectangleLines(square.x, square.y, square.size, square.size, GRAY);

        if (square.is_selected) {
            DrawRectangle(square.x, square.y, square.size, square.size, GREEN);
        }

        if (square.is_hovering) {
            DrawRectangle(square.x, square.y, square.size, square.size, BLUE);
        }
    }

    for (auto line: lines)
    {
        for (auto point: line.points)
        {
            DrawCircle(point.x, point.y, 5, MAGENTA);
        }
        std::vector<Vector2> points(line.points.size());
        for (int i = 0; i < points.size(); i++)
        {
            points[i] = Vector2{.x=(float)line.points[i].x, .y=(float)line.points[i].y};
        }
        DrawLineStrip(&points[0], line.points.size(), MAGENTA);
    }

    DrawText(mode_string.c_str(), 20, 20, 20, RED);
    EndDrawing();
}


#endif
