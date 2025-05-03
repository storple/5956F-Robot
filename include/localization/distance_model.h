#pragma once

#include "globals.h"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "pros/apix.h"
#include <cmath>
#include "localization/pose.h"
#include "localization/sensor.h"
#include "localization/utils.h"
#include <optional>
#include "localization/config.h"

namespace localization {
    class DistanceSensorModel : public Sensor {
        Length measured_distance = 0_m;
        pros::Distance * distance_sensor;

        float std_deviation = 0;
        Angle angle = 0_stDeg;

        Pose offsets;

        Pose rotated_offsets = {0_m,0_m,0_stDeg};

        // cached values
        double cosa;
        double sina;
        // 1 / trig function
        double rcosa;
        double rsina;

        Length horizontal_wall_length = wall_length; 
        Length vertical_wall_length = wall_length; 

        std::string name;

        public:
        // determines whether or not readings from this sensor are considered
        bool exit = false;
        bool disabled = false;
        
        void disable(){
            disabled = true;
        }

        void enable(){
            disabled = false;
        }


        DistanceSensorModel(const Pose offset, pros::Distance * distance_sensor,std::string name)
            : offsets(offset), distance_sensor(std::move(distance_sensor)), name(name) { }

        void update(Angle angle) override {
            // first check if the distance sensor is available, and if its not then fail non-destructively while still alerting user
            if(distance_sensor == nullptr || !distance_sensor->is_installed()){
                // not available, just set exit to true
                exit = true;
                    // printf("ONE OF THE DISTANCE SENSORS ARE NOT CONNECTED CORRECTLY!!");
                return;
            }

            const auto measured_mm = distance_sensor->get();

            this->measured_distance = from_mm(measured_mm);

            exit =
                // distance sensor doesn't measure anything
                measured_mm == 9999 ||
                // there is an obstacle which is not the field
                distance_sensor->get_object_size() < 65
                ||
                disabled
                ;

            this->std_deviation = (1.5_in).internal();
            
            this->angle = angle;
            const Angle offset_angle = this->angle + this->offsets.angle;
            this->cosa = units::cos(offset_angle).internal();
            this->sina = units::sin(offset_angle).internal();
            this->rcosa = 1/this->cosa;
            this->rsina = 1/this->sina;

            this->rotated_offsets = rotatePose(this->offsets, this->angle);

            // we will always compare all particles to two walls
            // one vertical and one horizontal
            // since the walls we check are always the same for both we can cache which wall we are checkign
            this->horizontal_wall_length = cosa > 0 ? wall_length : -wall_length;
            this->vertical_wall_length   = sina > 0 ? wall_length : -wall_length;

            // simplifies the math even further
            this->horizontal_wall_length = horizontal_wall_length - this->rotated_offsets.x;
            this->vertical_wall_length   = vertical_wall_length   - this->rotated_offsets.y;

            if(localization_settings::logging){
                // expected distance,confidence,std,exit
                std::cout << this->name << ":"
                    << this->measured_distance.convert(in) << ","
                    << this->distance_sensor->get_confidence() << ","
                    << this->std_deviation << ","
                    << (this->exit ? "true" : "false") << ","
                    << this->distance_sensor->get_object_size() << "\n";
            }
        }

        // returns x and y coordinates for which the distance sensor would match measurements
        // this is used to generate particles from distance sensor readings directly,
        // preventing a total system collapse in particles
        Point getExpected() override {
            if(exit){
                return {infinity()*m,infinity()*m};
            }
            // if we want to generate particles from the measurements themslves we can easily rearrange
            // to get the x and y values for which this measurement would be plausible
            // horizontal_wall_length - measured_distance * this->cosa  = point.x
            // vertical_wall_length -   measured_distance * this->sina  = point.y
            return {
                horizontal_wall_length - measured_distance * this->cosa,
                vertical_wall_length   - measured_distance * this->sina
            };
        }
        
        std::optional<float> p(const Point& point) override {
            if(exit){
                return std::nullopt;
            }

            // normal math is:
            // sensor_point = rotated_point + point
            //
            // (wall.x - sensor_point.x) / cos(angle)
            // (wall.y - sensor_point.y) / sin(angle)
            //
            // where wall.x / wall.y can be postive or negative depending on where we are facing
            //
            // this however can optimized by:
            // computing 1/ trig function to multiply instead of divide (multiplication is faster)
            //
            // expanding sensor_point:
            // (wall.x - (rotated_point.x + point.x)) / cos(angle)
            // ->
            // ((wall.x - rotated_point.x) - point.x) / cos(angle)
            // (wall.x - rotated_point.x) doesnt change point to point, therefore this can be cached as well 
            //
            // here horizontal_wall_length = (wall.x - rotated_point.x)
            // and vertical_wall_length = (wall.y - rotated_point.y)
            //

            const Length expected_distance = units::min(
                    (horizontal_wall_length - point.x)*this->rcosa,
                    (vertical_wall_length   - point.y)*this->rsina
                    );

            const Length difference = expected_distance - measured_distance;
            
            // if(units::abs(difference) > localization_settings::sensor_disparity_threshold){
            //     // if the distance sensors dont match at all by a long shot its probably a wrong measurement
            //     // so we just ignore it
            //     return std::nullopt;
            // }

            return
                // we can transform the standard normal distribution into any other normal distribution by a simple linear transformation
                // std_normal_dist.pdf((x - mean)/std_deviation) / std_deviation;
                // here the mean (expected value) is 0, since we would expect that from a perfect match
                cheapNormalDistribution(
                        difference.internal() / std_deviation
                        // including std_deviation can mess with the way weights are prioritized
                        // ) / std_deviation;
                        );
        }
        ~DistanceSensorModel() override = default;
    };
}
