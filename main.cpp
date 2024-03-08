#include <cmath>
#include <raylib.h>
#include <vector>
#include "vehicle.hpp"


const float SCALE = 5.0f;
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 600;
const float ROAD_LENGTH = WINDOW_WIDTH / SCALE;
const float R = ROAD_LENGTH / (2 * PI); // from the perimeter: p = 2 * pi * r

double previous_time = GetTime();
double current_time = 0.0;
float dt = 0.0f;

using namespace vehicle;

struct CirclePos
{
    Vec2 pos;
    float angle;
};

// draws the vehicle inside the circle with the right angle
void draw_vehicle(CirclePos circle_position)
{
    Color color = { 255, 0, 255, 255 };
    Rectangle rect = {
        SCALE * circle_position.pos.x,
        SCALE * circle_position.pos.y,
        SCALE * CAR_LENGTH,
        SCALE * CAR_WIDE
    };
    DrawRectanglePro(rect, {0, 0}, 90 + 180 / PI * circle_position.angle, color);
}


// converts the position to a circular position
CirclePos pos_to_circle(Vec2 pos)
{
    float x_offset =  WINDOW_WIDTH / (SCALE * 2.0f);
    float y_offset =  WINDOW_HEIGHT / (SCALE * 2.0f);
    float angle = pos.x / R;
    return {
        Vec2{
            R * cosf(angle) + x_offset,
            R * sinf(angle) + y_offset
        },
        angle
    };
}


int main(void)
{
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "raylib [core] example - basic window");
    int N = 10;
    std::vector<Vehicle> vehicles = spawn_cars_near(N, ROAD_LENGTH);

    std::vector<CirclePos> circular_positions(N);
    for (int i=0; i<N; i++)
    {
        circular_positions[i] = pos_to_circle(vehicles[i].position);
    }

    while (!WindowShouldClose())
    {
        // movement
        for (int i=0; i<N; i++)
        {
            float pos = vehicles[i].position.x + 10.0f * dt;
            // loop through the road, as its a circle
            if (pos > ROAD_LENGTH) {
                pos -= ROAD_LENGTH;
            }
            vehicles[i].position.x = pos;
            circular_positions[i] = pos_to_circle(vehicles[i].position);
        }
        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            for (int i=0; i<N; i++)
            {
                draw_vehicle(circular_positions[i]);
            }
        }
        EndDrawing();
        current_time = GetTime();
        dt = (float)(current_time - previous_time);
        previous_time = current_time;

    }

    CloseWindow();

    return 0;
}
