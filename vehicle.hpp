#ifndef VEHICLE_HPP
#define VEHICLE_HPP


#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <ranges>


namespace vehicle
{


struct Vec2
{
    float x;
    float y;
};


struct Vehicle
{
    Vec2 position;
    float acceleration;
    float velocity;
    float v_d;

    int lane;
};

// everything is in meters
const float CAR_LENGTH = 4.9;
const float CAR_WIDE = 1.94;

// IDM params
float a_max = 1.0f; // 0.8 to 2.5 m/s^2
float a_dec = 2.0f; // around 2 m/s^2
float delta_s_min = 2.0f; // m
float v_d = 120.0f / 3.6; // m/s
float mu = 4.0;
float T = 0.8; // 0.8 to 2.0 s
float alpha = 0.99;

// MOBIL params
float politiness = 1.0f;
float a_thresh = 0.1f; // m / s^2 - changing threshold
float min_gap = 5.0; // m - minimum gap needed to change lanes
float b_max = 4.0; // m / s^2 - maximum safe deceleration
float right_bias = 0.3; // m / s^2

float randf()
{
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}


/**
 * Returns the spawn position of N cards in the given road, equally spaced
 */
std::vector<Vehicle> spawn_cars(int N, std::vector<float> road_lengths, int n_lanes)
{
    std::vector<Vehicle> vehicles(N * n_lanes);
    for (int l = 0; l < n_lanes; l++)
    {
        float road_length = road_lengths[l];
        float x_step = (road_length - 2 * CAR_LENGTH) / N;
        for (int i=1; i < N+1; i++)
        {
            Vec2 pos = { .x = i * x_step, .y = 20 };
            vehicles[(i-1) + l * N].position = pos;
            vehicles[(i-1) + l * N].acceleration = 0.2f * randf() + 1.0f;
            vehicles[(i-1) + l * N].velocity = v_d * (0.05f * randf() + 0.95f);
            vehicles[(i-1) + l * N].v_d = v_d;
            vehicles[(i-1) + l * N].lane = l;
        }
    }
    return vehicles;
}


std::vector<Vehicle> spawn_cars_near(int N, int n_lanes)
{
    std::vector<Vehicle> vehicles(N);
    float x_step = delta_s_min + CAR_LENGTH + 2.0f * randf();
    for (int l = 0; l < n_lanes; l++)
    {
        for (int i=0; i < N; i++)
        {
            Vec2 pos = { .x = i * x_step, .y = 20 };
            vehicles[i].position = pos;
            vehicles[i].acceleration = 0.2f * randf() + 1.0f;
            vehicles[i].velocity = v_d * (0.05f * randf() + 0.95f);
            vehicles[i].v_d = v_d;
            vehicles[i-1].lane = l;
        }
    }
    return vehicles;
}


template<typename SortFn>
std::vector<Vehicle> get_vehicles_in_lane(Vehicle v, const std::vector<Vehicle> vehicles, SortFn fn)
{
    auto vehicles_in_lane_views = vehicles | std::views::filter([&v](Vehicle ov){return ov.lane == v.lane;});
    auto vehicles_in_lane = std::vector<Vehicle>(vehicles_in_lane_views.begin(), vehicles_in_lane_views.end());
    std::ranges::sort(vehicles_in_lane, fn);
    return vehicles_in_lane;
}


/**
 * Finds the leader vector taking into consideration the car lane
 */
Vehicle find_leader(Vehicle v, const std::vector<Vehicle>& vehicles)
{
    auto vehicles_in_lane = get_vehicles_in_lane(v, vehicles, [](Vehicle v1, Vehicle v2){return v1.position.x < v2.position.x;});
    auto it = std::ranges::find_if(vehicles_in_lane, [&v](Vehicle ov) {return ov.position.x > v.position.x;});
    Vehicle leader = it != vehicles_in_lane.end() ? *it : vehicles_in_lane[0];
    return leader;
}


Vehicle find_follower(Vehicle v, const std::vector<Vehicle>& vehicles)
{
    auto vehicles_in_lane = get_vehicles_in_lane(v, vehicles, [](Vehicle v1, Vehicle v2){return v1.position.x > v2.position.x;});
    auto it = std::ranges::find_if(vehicles_in_lane, [&v](Vehicle ov) {return ov.position.x < v.position.x;});
    Vehicle follower = it != vehicles_in_lane.end() ? *it : vehicles_in_lane[0];
    return follower;
}


// Intelligent Driver Model
// https://traffic-simulation.de/info/info_IDM.html
float IDM(float delta_s, float v, float delta_v, float v_d)
{
    float delta_s_star = delta_s_min + v * T + (v * delta_v) / (2 * std::sqrt(a_max * a_dec));
    float a_idm = a_max * (1 - std::pow(v / v_d, mu) - std::pow(delta_s_star / delta_s, 2.0f));
    return a_idm;
}


// Constant-Acceleration Heuristic
// https://arxiv.org/pdf/0912.3613.pdf pages 4 and 5
float CAH(float delta_s, float v, float v_l, float a_l)
{
    float delta_v = v - v_l;
    float a_tilde = std::min(a_l, a_max);

    if (v_l * delta_v <= -2 * delta_s * a_tilde) {
        return (v * v * a_tilde) / (v_l * v_l - 2 * delta_s * a_tilde);
    } else {
        float step = delta_v < 0 ? 0 : 1;
        return a_tilde - (delta_v * delta_v * step) / (2 * delta_s);
    }
}


// Adaptive Cruise Control
// https://arxiv.org/pdf/0912.3613.pdf pages 5 and 6
float ACC(float delta_s, float v, float v_leader, float a_leader, float v_desired)
{
    float delta_v = v - v_leader;
    float a_idm = IDM(delta_s, v, delta_v, v_desired);
    float a_cah = CAH(delta_s, v, v_leader, a_leader);

    if (a_idm >= a_cah) {
        return a_idm;
    } else {
        return (1 - alpha) * a_idm + alpha * ( a_cah + a_dec * std::tanh((a_idm - a_cah)/a_dec) );
    }
}


/**
 * MOBIL lane change algorithm
 * https://www.mtreiber.de/publications/MOBIL_TRB.pdf
 *
 * Defines if a car should change its lane.
 *
 * @param Vehicle me: the vehicle which we want to calculate whether to change lane or not
 * @param Vehicle new_leader: the new leader if the car changes lane
 * @param Vehicle new_follower: the new follower (vehicle which will be before us) if the car changes lane
 * @param Vehicle old_leader: the vehicle current leader
 * @param Vehicle old_follower: the current follower (vehicle which is be before us)
 */
bool MOBIL(Vehicle me, Vehicle new_leader, Vehicle new_follower, Vehicle old_leader, Vehicle old_follower)
{
    // check if we have enough space
    if (std::abs(me.position.x - new_leader.position.x) < min_gap || std::abs(me.position.x - new_follower.position.x) < min_gap) {
        return false;
    }

    // safety criterion: equation 2: ã_n >= -b_safe
    float a_tilde_me = ACC(std::abs(me.position.x - new_leader.position.x),
                           me.velocity,
                           new_leader.velocity,
                           new_leader.acceleration,
                           me.v_d); // new acceleration if lane change occurs
    float a_tilde_new_follower = ACC(std::abs(me.position.x - new_follower.position.x),
                                     new_follower.velocity,
                                     me.velocity,
                                     me.acceleration,
                                     new_follower.v_d); // new follower new acceleration if change occurs
    if (a_tilde_me < -b_max || a_tilde_new_follower < -b_max) {
        return false;
    }

    // equations 6 / 7
    float a_me = ACC(std::abs(me.position.x - new_leader.position.x),
                           me.velocity,
                           old_leader.velocity,
                           old_leader.acceleration,
                           me.v_d); // new acceleration if lane change occurs
    float a_new_follower = ACC(std::abs(me.position.x - new_follower.position.x),
                                     new_follower.velocity,
                                     new_leader.velocity,
                                     new_leader.acceleration,
                                     new_follower.v_d); // new follower new acceleration if change occurs
    bool is_right = new_leader.lane > old_leader.lane;
    int sign = is_right ? 1 : -1;
    return a_tilde_me - a_me + politiness * (a_tilde_new_follower - a_new_follower) > a_thresh - sign * right_bias;
}


// Returns the speed at time t + dt (integrates the acceleration)
float integrate_acc(float v, float acc, float dt)
{
    return v + acc * dt;
}


// Returns the position at time t + dt (integrates the velocity)
float integrate_v(float s, float v, float acc, float dt)
{
    return s + v * dt + 0.5f * acc * dt * dt;
}


void move_vehicle(int vehicle_index, std::vector<Vehicle> &vehicles, float dt, float road_length)
{
    Vehicle vehicle = vehicles[vehicle_index];
    Vehicle leader = find_leader(vehicle, vehicles);
    // due to circular movement, delta_s could be negative, so we take
    // the absolute value
    float delta_s = (leader.position.x > vehicle.position.x
        ? leader.position.x - vehicle.position.x
        : road_length - vehicle.position.x + leader.position.x
    ) - CAR_LENGTH;

    float a = ACC(delta_s, vehicle.velocity, leader.velocity, leader.acceleration, vehicle.v_d);
    float v = integrate_acc(vehicle.velocity, vehicle.acceleration, dt);
    float s = integrate_v(vehicle.position.x, vehicle.velocity, vehicle.acceleration, dt);
    vehicles[vehicle_index].acceleration = a;
    vehicles[vehicle_index].velocity = v < 0 ? 0 : v;
    vehicles[vehicle_index].position.x = s;
}

};

#endif

