#include <algorithm>
#include <cmath>
#include <cstdio>
#include <execution>
#include <raylib.h>
#include <string>
#include <sys/types.h>
#include <vector>
#include <fstream>

#include "utils.hpp"
#include "vehicle.hpp"
#include "detector.hpp"
#include "speed_limit.hpp"
#include "nlohmann/json.hpp"
#include "argparse/argparse.hpp"

using json = nlohmann::json;


#define R_OFFSET 5.0f
#define LANE_CHANGE_TIMEOUT 2.5f
#define MAX_JSON_LENGTH 50


struct Args : public argparse::Args {
    std::string &filepath = kwarg("f,filepath", "Output json file");
    std::string &det_prefix = kwarg("d, det-prefix", "Prefix to detectors file");
    int &n_lanes = kwarg("n,n-lanes", "The number of lanes to simulate").set_default(3);
    int &n_cars = kwarg("c,n-cars", "The number of cars per lane").set_default(20);
    bool &no_control = flag("no-control", "don't use speed control");
};


const float SCALE = 4.0f;
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 600;
const float R = 65.0f;

double previous_time = GetTime();
double current_time = 0.0;
float dt = 0.0f;

using namespace vehicle;

struct CirclePos
{
    Vec2 pos;
    float angle;
};


float get_road_length(int lane)
{
    return 2 * PI * (R - lane * R_OFFSET);
}


void draw_detectors(std::vector<Detector> ds)
{
    for (int i=0; i<ds.size(); i++)
    {
        Detector d = ds[i];
        Color color = { 255, 0, 0, 125 };
        float x_offset =  WINDOW_WIDTH / (SCALE * 2.0f);
        float y_offset =  WINDOW_HEIGHT / (SCALE * 2.0f);
        float angle = d.u / R;
        Vec2 pos = Vec2{
            R * cosf(angle) + x_offset,
            R * sinf(angle) + y_offset
        };
        Rectangle rect = {
            SCALE * pos.x,
            SCALE * pos.y,
            3 * SCALE * CAR_LENGTH,
            SCALE * CAR_WIDE
        };
        DrawRectanglePro(rect, {0, 0}, - 180 + 180 / PI * angle, color);
    }
}


// draws the vehicle inside the circle with the right angle
void draw_vehicle(CirclePos circle_position, int i, vehicle::Vehicle v)
{

    u_int8_t hue = (u_int8_t)(std::min(1.0f, v.velocity / 100) * 255);
    Color color = HsvToRgb({hue, 255, 255});
    //Color color = v.type == CAR ? (Color){0, 255, 0, alpha} : (Color){ 0, 0, 255, alpha };
    Rectangle rect = {
        SCALE * circle_position.pos.x,
        SCALE * circle_position.pos.y,
        SCALE * CAR_LENGTH,
        SCALE * CAR_WIDE
    };
    DrawRectanglePro(rect, {0, 0}, 90 + 180 / PI * circle_position.angle, color);
    if (v.type == TRUCK) {
        DrawCircle(rect.x - 5, rect.y + 5, 5, MAGENTA);
    }
    DrawText(TextFormat("%i", i), rect.x, rect.y, 15, ORANGE);
}


// converts the position to a circular position
CirclePos pos_to_circle(Vec2 pos, int lane)
{
    float x_offset =  WINDOW_WIDTH / (SCALE * 2.0f);
    float y_offset =  WINDOW_HEIGHT / (SCALE * 2.0f);
    float radius = R - R_OFFSET * lane;
    float angle = pos.x / radius;
    return {
        Vec2{
            radius * cosf(angle) + x_offset,
            radius * sinf(angle) + y_offset
        },
        angle
    };
}



void draw_info(std::vector<Vehicle> vehicles, std::vector<Detector> ds)
{
    DrawText(TextFormat("Last car desired velocity: %.02f km/h", vehicles[vehicles.size()-1].v_d * 3.6), 10, 10, 20, RED);
    DrawText(TextFormat("First car velocity: %.02f km/h", vehicles[0].velocity * 3.6), 10, 30, 20, RED);
    DrawText(TextFormat("Last car velocity: %.02f km/h", vehicles[vehicles.size()-1].velocity * 3.6), 10, 50, 20, RED);

    int offset = 10;
    for (int i=0; i<ds.size(); i++) {
        if (ds[i].velocities.size() == 0) {
            continue;
        }
        auto d = ds[i];

        DrawText(TextFormat("Detector %i velocity: %.02f km/h", i, d.velocities[d.velocities.size()-1]), WINDOW_WIDTH - 350, offset, 20, RED);
        offset += 20;
    }
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

    DrawText(TextFormat("Input %s", s.c_str()), 10, 70, 20, RED);
}


int main(int argc, char* argv[])
{
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "raylib [core] example - basic window");

    auto args = argparse::parse<Args>(argc, argv);

    int n_lanes = args.n_lanes;
    int n_cars = args.n_cars;
    int N = n_lanes * n_cars;
    std::vector<float> road_lengths = std::vector<float>(n_lanes);
    for (int i = 0; i < n_lanes; i++) {
        road_lengths[i] = get_road_length(i);
    }
    std::vector<Vehicle> vehicles = spawn_cars(n_cars, road_lengths, n_lanes, 0.7f);

    std::vector<CirclePos> circular_positions(N);
    for (int i=0; i<N; i++) {
        circular_positions[i] = pos_to_circle(vehicles[i].position, vehicles[i].lane);
    }

    json j;
    // only save when json_dt > json_period
    float json_dt = 0;
    float json_period = 1.5f;
    int iters = 0;
    Detector d1(0, json_period, vehicles);
    Detector d2(0.3f * road_lengths[0], json_period, vehicles);
    Detector d3(0.6f * road_lengths[0], json_period, vehicles);
    std::vector<Detector> ds = {d1, d2, d3};

    SpeedLimit sp1(0.25f * road_lengths[0], 1, vehicles);
    SpeedLimit sp2(0.5f * road_lengths[0], 20 / 3.6, vehicles);
    SpeedLimit sp3(0.5f * road_lengths[0], 35 / 3.6, vehicles);
    SpeedLimit sp4(0.5f * road_lengths[0], 50 / 3.6, vehicles);
    while (!WindowShouldClose())
    {
        // movement
        for (int i=0; i<N; i++)
        {
            Vehicle* v = &vehicles[i];
            float road_length = road_lengths[v->lane];
            vehicle::move_vehicle(i, vehicles, dt, road_length);
            // loop through the road, as its a circle
            if (v->position.x > road_length) {
                v->position.x = 0;
            }
            circular_positions[i] = pos_to_circle(vehicles[i].position, vehicles[i].lane);
        }

        if (n_lanes > 1) {
            std::for_each(
                std::execution::par,
                vehicles.begin(),
                vehicles.end(),
                [&vehicles, &road_lengths](auto& vehicle) {
                    auto t = MOBIL(vehicle, vehicles, road_lengths);
                    if(std::get<0>(t)) {
                        vehicle.lane = std::get<1>(t);
                        vehicle.position.x = std::get<2>(t);
                        vehicle.lane_change_timeout = LANE_CHANGE_TIMEOUT;
                    }
                }
            );
        }

        if (iters < 15) {
            sp1.apply(vehicles);
        } else if (iters == 15) {
            sp1.remove_speed_limit(vehicles);
            if (!args.no_control) {
                sp2.apply_all(vehicles);
            }
        } else if (!args.no_control && iters == 20) {
            sp2.remove_speed_limit(vehicles);
            sp3.apply_all(vehicles);
        } else if (!args.no_control && iters == 30) {
            sp3.remove_speed_limit(vehicles);
            sp4.apply_all(vehicles);
        } else if (!args.no_control && iters == 40) {
            sp4.remove_speed_limit(vehicles);
        }


        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            for (int i=0; i<N; i++)
            {
                draw_vehicle(circular_positions[i], i, vehicles[i]);
            }
            draw_info(vehicles, ds);
            input_desired_velocity(vehicles[vehicles.size()-1]);
            draw_detectors(ds);
        }
        EndDrawing();
        current_time = GetTime();
        dt = (float)(current_time - previous_time);
        previous_time = current_time;

        // tse
        for (Detector& d : ds) {
            d.calc_tse(vehicles, dt);
        }
        // vehicles json
        json_dt += dt;
        for (int i = 0; i < vehicles.size(); i++)
        {
            vehicles[i].lane_change_timeout -= dt;
            if (vehicles[i].lane_change_timeout < 0) {
                vehicles[i].lane_change_timeout = 0;
            }
        }

        // save data to json
        if (json_dt > json_period) {
            iters++;
            printf("iter %d\n", iters);
            std::string key = std::to_string(iters);
            j[key] = json::array();
            for (int i = 0; i < vehicles.size(); i++) {
                Vehicle v = vehicles[i];
                j[key].push_back({ {"id", i }, {"u", v.position.x}, {"lane", v.lane}, {"speed", v.velocity } });
            }
            if (iters == MAX_JSON_LENGTH) {
                printf("JSON completed\n");
                for (int i=0; i < ds.size(); i++) {
                    std::string detector_json = ds[i].to_json().dump();
                    std::ostringstream filename;
                    filename << args.det_prefix << "detector" << i << ".json";
                    std::ofstream out(filename.str());
                    out << detector_json;
                    out.close();
                }
                std::ofstream out_veh(args.filepath);
                out_veh << j.dump();
                out_veh.close();
            }
            json_dt = 0;
        }
    }

    CloseWindow();

    return 0;
}
