// #include 
#include "globals.h"
#include <vector>
#include <math.h>

using namespace Robot;
using namespace Robot::Globals;

//
// utils for the various autonomous routines
// please credit me on your notebook ok please thank you

const float HALF_WALL_LENGTH = 70.4;

enum SensorOrientation { HORIZONTAL, VERTICAL };
enum Wall { LEFT, TOP, RIGHT, BOTTOM };

struct distanceSensor {
    pros::Distance * sensor;

    SensorOrientation orientation;
    
    // both in inches
    float x_offset;
    float y_offset;
    // in degrees
    float angle_offset;
};

// pros::Distance distance_sensor;
// pros::Distance wall_sensor;
// pros::Distance wall_sensor_back;

std::vector<distanceSensor> distance_sensors {
    {&distance_sensor, HORIZONTAL, 0,0,0},
    {&wall_sensor, VERTICAL, 0,0,0},
    {&wall_sensor_back, VERTICAL, 0,0,0},
};
struct distance_sensor_reading {
    float measured_distance;
    SensorOrientation orientation;
    
    // both in inches
    float x_offset;
    float y_offset;
    // in degrees
    float angle_offset;
};

struct generated_coordinate {
    float x;
    float y;
    SensorOrientation orientation;
};

// returns possible x and y coordinates of expected movement in inches
generated_coordinate get_expected_coordinates(lemlib::Pose pose, float angle, distance_sensor_reading reading){
    const float offset_angle = (angle + reading.angle_offset) * (M_PI / 180);

    const float cosa = cos(offset_angle);
    const float sina = sin(offset_angle);
    
    float x = 0;
    float y = 0;

    if(cosa > 0){
        x = HALF_WALL_LENGTH - reading.measured_distance * cosa;
    }else{
        x = -HALF_WALL_LENGTH - reading.measured_distance * cosa;
    }
    if(sina > 0){
        y = HALF_WALL_LENGTH - reading.measured_distance * sina;
    }else{
        y = -HALF_WALL_LENGTH - reading.measured_distance * sina;
    }
    return {x,y,reading.orientation};
}

bool outsideOfField(float x,float y){
    return x > HALF_WALL_LENGTH || x < -HALF_WALL_LENGTH || y < -HALF_WALL_LENGTH || y > HALF_WALL_LENGTH;
}

// sets the distance
// returns false if it was not able to reset position
bool distance_reset_position(){
    lemlib::Pose current_pose = chassis.getPose();
    const float angle = 90 - current_pose.theta;

    int horizontal_sensors = 0;
    int vertical_sensors = 0;

    std::vector<distance_sensor_reading> readings;

    for(auto && sensor : distance_sensors){
        const int measured_distance = sensor.sensor->get();
        // here we declare we want to use this distance sensor, only if 
        // there is a measurement and the measurement is accurate
        if(measured_distance != 9999 && measured_distance > 0) {
            if(sensor.orientation == HORIZONTAL){
                horizontal_sensors++;
            }
            else{
                vertical_sensors++;
            }
            readings.emplace_back(
                    measured_distance / 25.4,
                    sensor.orientation,
                    sensor.x_offset,
                    sensor.y_offset,
                    sensor.angle_offset
                    );
        }
    }
    // we only want to continue if there is at least one horizontal and one vertical reading 
    if(horizontal_sensors == 0 || vertical_sensors == 0){
        return false;
    }

    std::vector<generated_coordinate> generated_coordinates;

    for(auto & reading : readings){
        const generated_coordinate generated = get_expected_coordinates(current_pose, angle, reading);
        generated_coordinates.push_back(generated);
    }
    
    std::vector<std::pair<float,float>> final_coordinates;

    // get the final measurements
    for(size_t i = 0;i < generated_coordinates.size();i++){
        for(size_t j = i+1;j < generated_coordinates.size();j++){
            // not using two sensors with the same orientation
            if(generated_coordinates[i].orientation != generated_coordinates[j].orientation){
                // y means top or bottom wall
                // x means left or right
                //
                if(!outsideOfField( generated_coordinates[i].x, generated_coordinates[j].y )){
                    final_coordinates.emplace_back( generated_coordinates[i].x, generated_coordinates[j].y);
                }

                if(!outsideOfField( generated_coordinates[j].x, generated_coordinates[i].y )){
                    final_coordinates.emplace_back( generated_coordinates[j].x, generated_coordinates[i].y);
                }
            }
        }
    }

    // takes average of all readings
    // this might happen if there are more than 2 readings available, so we take the average to have the most accurate position
    float fx = 0, fy = 0;
    for(auto && coordinate : final_coordinates){
        fx += coordinate.first;
        fy += coordinate.second;
    }

    if(final_coordinates.empty()){
        return false;
    }

    float ffx = fx / static_cast<float>(final_coordinates.size());
    float ffy = fy / static_cast<float>(final_coordinates.size());

    // shift y coordinate by 48 inches
    ffy += 48;

    chassis.setPose(ffx,ffy,chassis.getPose().theta);

    return true;
}
