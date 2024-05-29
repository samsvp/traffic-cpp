#ifndef SPEED_LIMIT_HPP
#define SPEED_LIMIT_HPP

#include <vector>
#include "vehicle.hpp"

using namespace vehicle;


class SpeedLimit
{
public:
    float u;
    float v_d;
    SpeedLimit(float u, float v_d, std::vector<Vehicle> vs) : u(u), v_d(v_d)
    { }


    void apply(std::vector<Vehicle>& vs)
    {
        float offset = 10.5f;
        for (int i = 0; i < vs.size(); i++)
        {
            auto& v = vs[i];
            if (v.position.x < u && v.position.x > u - offset) {
                v.v_d = v_d;
            }
        }
    }


    void apply_all(std::vector<Vehicle>& vs)
    {
        for (int i = 0; i < vs.size(); i++)
        {
            auto& v = vs[i];
            v.v_d = v_d;
        }

    }


    void remove_speed_limit(std::vector<Vehicle>& vs)
    {
        for (int i = 0; i < vs.size(); i++)
        {
            auto& v = vs[i];
            v.v_d = v.type == CAR ? v_d_car : v_d_truck;
        }
    }
};


#endif
