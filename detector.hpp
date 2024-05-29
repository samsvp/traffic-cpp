#ifndef DETECTOR_HPP
#define DETECTOR_HPP


#include "nlohmann/json.hpp"
#include "vehicle.hpp"
#include <map>
#include <numeric>
#include <string>
#include <vector>

using namespace vehicle;
using ordered_json = nlohmann::ordered_json;

class Detector
{

    public:
        float u;
        std::vector<float> velocities;
        std::vector<float> flows;
        std::vector<float> densities;
        Detector(float u, float period, std::vector<Vehicle> vs) : u(u), period(period)
        {
            update_vehicle_pos(vs);
        }


        void calc_tse(std::vector<Vehicle> vs, float dt)
        {
            // get vehicles velocity which just passed through u
            std::vector<float> velocities;
            float offset = 1.5f;
            for (int i = 0; i < vs.size(); i++)
            {
                auto v = vs[i];
                if (v.position.x < u + offset && v.position.x > u - offset) {
                    velocities.push_back(v.velocity);
                }
            }
            float velocity_acc = std::accumulate(velocities.begin(), velocities.end(), 0.0f);
            velocity_sum += velocity_acc;
            vehicle_count += velocities.size();
            passed_dt += dt;
            // update tse history
            if (passed_dt >= period) {
                float v = velocity_sum / vehicle_count;
                float flow = vehicle_count / passed_dt;
                this->velocities.push_back(3.6 * v);
                flows.push_back(3600 * flow);
                if (v > 0) {
                    densities.push_back(1000 * flow / v);
                } else {
                    densities.push_back(0);
                }

                velocity_sum = 0;
                vehicle_count = 0;
                passed_dt = 0;
            }

            update_vehicle_pos(vs);
        }


        ordered_json to_json() const
        {
            ordered_json j;
            for (int i=0; i < velocities.size(); i++)
            {
                std::string key = std::to_string(i);
                j[key] = {{"velocity", velocities[i]}, {"flow", flows[i]}, {"dens", densities[i]}};
            }
            return j;
        }


    private:
        float velocity_sum = 0;
        float vehicle_count = 0;
        // when passed_dt > period calculate tse
        float passed_dt = 0;
        float period;
        std::map<int, float> last_vehicles_positions;

        void update_vehicle_pos(std::vector<Vehicle> vs)
        {
            for (int i=0; i<vs.size(); i++) {
                last_vehicles_positions[i] = vs[i].position.x;
            }
        }
};

#endif
