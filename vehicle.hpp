#ifndef VEHICLE_HPP
#define VEHICLE_HPP


#include <cmath>
#include <cstdlib>
#include <vector>


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
};

// everything is in meters
const float CAR_LENGTH = 4.9;
const float CAR_WIDE = 1.94;

// IDM params
float a_max = 2.0f; // 0.8 to 2.5 m/s^2
float a_dec = 2.0f; // around 2 m/s^2
float delta_s_min = 2.0f; // m
float v_d = 120.0f / 3.6; // m/s
float mu = 4.0;
float T = 1.8; // 0.8 to 2.0 s
float alpha = 0.99;


float randf()
{
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}


/**
 * Returns the spawn position of N cards in the given road, equally spaced
 */
std::vector<Vehicle> spawn_cars(int N, float road_length)
{
    std::vector<Vehicle> vehicles(N);
    float x_step = (road_length - 2 * CAR_LENGTH) / N;
    for (int i=1; i < N+1; i++)
    {
        Vec2 pos = { .x = i * x_step, .y = 20 };
        vehicles[i-1].position = pos;
        vehicles[i-1].acceleration = randf() + 1.0f;
        vehicles[i-1].velocity = v_d * (0.2f * randf() + 0.8f);
    }
    return vehicles;
}


std::vector<Vehicle> spawn_cars_near(int N, float road_length)
{
    std::vector<Vehicle> vehicles(N);
    float x_step = 1.1 * CAR_LENGTH;
    for (int i=0; i < N; i++)
    {
        Vec2 pos = { .x = i * x_step, .y = 20 };
        vehicles[i].position = pos;
        vehicles[i].acceleration = randf() + 1.0f;
        vehicles[i].velocity = v_d * (0.2f * randf() + 0.8f);
    }
    return vehicles;
}


// Intelligent Driver Model
// https://traffic-simulation.de/info/info_IDM.html
float IDM(float delta_s, float v, float delta_v)
{
    float delta_s_star = delta_s_min + v * T + (v * delta_v) / (2 * std::sqrt(a_max * a_dec));
    float a_idm = a_max * (1 - std::pow(v / v_d, mu) - std::pow(delta_s_star, delta_s));
    return a_idm;
}


// Constant-Acceleration Heuristic
// https://arxiv.org/pdf/0912.3613.pdf pages 4 and 5
float CAH(float delta_s, float v, float v_l, float a_l)
{
    float delta_v = v - v_l;
    float a_tilde = std::min(a_l, a_max);

    if (v_l * (delta_v) <= -2 * delta_s * a_tilde) {
        return v * v / (v_l * v_l - 2 * delta_s * a_tilde);
    } else {
        float step = delta_v < 0 ? 0 : 1;
        return a_tilde - (delta_v * delta_v * step) / (2 * delta_s);
    }
}


// Adaptive Cruise Control
// https://arxiv.org/pdf/0912.3613.pdf pages 5 and 6
float ACC(float delta_s, float v, float delta_v, float v_l, float a_l)
{
    float a_idm = IDM(delta_s, v, delta_v);
    float a_cah = CAH(delta_s, v, v_l, a_l);

    if (a_idm >= a_cah) {
        return a_idm;
    } else {
        return (1 - alpha) * a_idm + alpha * ( a_cah + a_dec * std::tanh((a_idm - a_cah)/a_dec) );
    }
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

};

#endif

