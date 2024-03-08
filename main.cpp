#include <cmath>
#include <cstdio>
#include <raylib.h>
#include <string>
#include <vector>
#include "vehicle.hpp"


const float SCALE = 4.0f;
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
void draw_vehicle(CirclePos circle_position, int i)
{
    Color color = { 0, 0, 255, 255 };
    Rectangle rect = {
        SCALE * circle_position.pos.x,
        SCALE * circle_position.pos.y,
        SCALE * CAR_LENGTH,
        SCALE * CAR_WIDE
    };
    DrawRectanglePro(rect, {0, 0}, 90 + 180 / PI * circle_position.angle, color);
    DrawText(TextFormat("%i", i), rect.x, rect.y, 15, ORANGE);
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



void draw_info(std::vector<Vehicle> vehicles)
{
    DrawText(TextFormat("Last car desired velocity: %.02f km/h", vehicles[vehicles.size()-1].v_d * 3.6), 10, 10, 20, RED);
    DrawText(TextFormat("Last car velocity: %.02f km/h", vehicles[vehicles.size()-1].velocity * 3.6), 10, 30, 20, RED);
}


void input_desired_velocity(Vehicle& v)
{
    static std::string s = "";
    if (IsKeyPressed(KEY_ONE)) s += "1";
    if (IsKeyPressed(KEY_TWO)) s += "2";
    if (IsKeyPressed(KEY_THREE)) s += "3";
    if (IsKeyPressed(KEY_FOUR)) s += "4";
    if (IsKeyPressed(KEY_FIVE)) s += "5";
    if (IsKeyPressed(KEY_SIX)) s += "6";
    if (IsKeyPressed(KEY_SEVEN)) s += "7";
    if (IsKeyPressed(KEY_EIGHT)) s += "8";
    if (IsKeyPressed(KEY_NINE)) s += "9";
    if (IsKeyPressed(KEY_ZERO)) s += "0";
    if (IsKeyPressed(KEY_BACKSPACE)) {
        if (s.size() > 0) {
            s = s.substr(0, s.size() - 1);
        }
    }
    if (IsKeyPressed(KEY_PERIOD)) s += ".";
    if (IsKeyPressed(KEY_ENTER)) {
        v.v_d = std::stof(s) / 3.6;
        s = "";
    }

    DrawText(TextFormat("Input %s", s.c_str()), 10, 50, 20, RED);
}


int main(void)
{
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "raylib [core] example - basic window");
    int N = 18;
    std::vector<Vehicle> vehicles = spawn_cars(N, ROAD_LENGTH);

    std::vector<CirclePos> circular_positions(N);
    printf("\nInitial positions:\n");
    for (Vehicle v: vehicles) {
        printf("Pos X: %f, velocity: %f, acc: %f\n", v.position.x, v.velocity, v.acceleration);
    }
    for (int i=0; i<N; i++)
    {
        circular_positions[i] = pos_to_circle(vehicles[i].position);
    }

    while (!WindowShouldClose())
    {
        // movement
        for (int i=0; i<N; i++)
        {
            vehicle::move_vehicle(i, vehicles, dt, ROAD_LENGTH);
            Vehicle* v = &vehicles[i];
            // loop through the road, as its a circle
            if (v->position.x > ROAD_LENGTH) {
                v->position.x = 0;
            }
            circular_positions[i] = pos_to_circle(vehicles[i].position);
        }
        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            for (int i=0; i<N; i++)
            {
                draw_vehicle(circular_positions[i], i);
            }
            draw_info(vehicles);
            input_desired_velocity(vehicles[vehicles.size()-1]);
        }
        EndDrawing();
        current_time = GetTime();
        dt = (float)(current_time - previous_time);
        previous_time = current_time;
    }

    CloseWindow();

    return 0;
}
