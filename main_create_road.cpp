#include <cstdio>
#include <raylib.h>
#include <vector>

#include "create_road.hpp"
#include "road_types.hpp"


int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    Vector2 window_size = { .x=screenWidth, .y=screenHeight };

    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

    SetTargetFPS(60);

    std::vector<Square> grid;
    int square_size = 10;
    for (int y = 0; y < screenHeight; y+=square_size) {
        for (int x = 0; x < screenWidth; x+=square_size) {
            grid.push_back(Square{.x=x, .y=y, .size=square_size,.is_selected=false, .is_hovering=false});
        }
    }

    std::vector<RoadLine> lines;
    RoadLine* current_line = NULL;

    Mode current_mode = MODE_DEFAULT;

    while (!WindowShouldClose())
    {
        if (IsKeyReleased(KEY_C)) {
            for (int i = 0; i < grid.size(); i++) {
                grid[i].is_selected = false;
            }
        }
        if (IsKeyReleased(KEY_D)) {
            current_mode = MODE_DEFAULT;
        }
        if (IsKeyReleased(KEY_L)) {
            current_mode = MODE_LINE;
        }
        switch (current_mode) {
            case MODE_DEFAULT:
                check_grid_collisions(grid);
                break;
            case MODE_LINE:
                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    if (current_line == NULL) {
                        RoadLine line = create_line(GetMousePosition());
                        lines.push_back(line);
                        current_line = &lines[lines.size()-1];
                    }
                } else if (IsMouseButtonUp(MOUSE_BUTTON_LEFT)) {
                    if (current_line != NULL) {
                        append_to_line(GetMousePosition(), current_line);
                        current_line = NULL;
                    }
                }
                break;
            default:
                break;
        }

        draw(grid, lines, current_mode);
    }
    return 0;
}
