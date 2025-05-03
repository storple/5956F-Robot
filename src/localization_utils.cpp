#include "localization_utils.h"
#include "globals.h"

namespace localization {

    // to mirror autons we want to define which corner the auton starts at and which one
    // is the current one
    // lets define
    // neg blue    |     neg red
    // -------------------------
    // pos blue    |    pos red
    //
    // particle filter should have points initialized to the correct corner
    // particle filter should know the real angle
    //
    // lemlib should think it is in the corner it was made in
    // we shoudl transform the real angles to match what they would be in the original mode
    //
    // angle: setPose -> accurate => particle_filter -- transformed to match orignal mode -> lemlib
    // point: setPose -> accurate => particle_filter -- transormed to match original mode -> lemlib
    
    
    // false -> negative
    // true -> positive
    // std::map<auton_corner_t,std::pair<bool,bool>> modifiers{
    //     {neg_red ,{false,true}},
    //     {neg_blue,{true ,true}},
    //     {pos_red ,{false,false}},
    //     {pos_blue,{true, false}}
    // };
    //
    // void setStartCorner(auton_corner_t new_corner){start_corner = new_corner;}

    // void setCurrCorner(auton_corner_t new_corner){
    //     curr_corner = new_corner;
    //     if(curr_corner == neg_red || curr_corner == pos_red){
    //         alliance = red;
    //     }
    //     else{
    //         
    //         alliance = blue;
    //     }
    // }

    // float getNewAngle(float theta){
    //     bool x_mod = modifiers[start_corner].first  ^ modifiers[start_corner].first;      
    //     bool y_mod = modifiers[start_corner].second ^ modifiers[start_corner].second;      
    //
    //     if(x_mod && y_mod){
    //         return theta + 180;
    //     }
    //     else if(x_mod){
    //         return 180 - theta;
    //     }else if(y_mod){
    //         return 360 - theta;
    //     }else{
    //         return theta;
    //     }
    // }
    //
    // localization::Point getNewPoint(localization::Point curr_point){
    //     // if the x is different then x changes
    //     bool x_mod = modifiers[start_corner].first  ^ modifiers[start_corner].first;      
    //     bool y_mod = modifiers[start_corner].second ^ modifiers[start_corner].second;      
    //
    //     return {x_mod ? -curr_point.x : curr_point.x,
    //             y_mod ? -curr_point.y : curr_point.y};
    //     }
    //
    // 
    // localization::Pose getNewPose(localization::Pose curr_pose){
    //     if(start_corner == curr_corner) return curr_pose;
    //
    //     // if the x is different then x changes
    //     bool x_mod = modifiers[start_corner].first  ^ modifiers[start_corner].first;      
    //     bool y_mod = modifiers[start_corner].second ^ modifiers[start_corner].second;      
    //
    //     Angle new_angle = curr_pose.angle; 
    //     // Angle new_angle; 
    //
    //     if(x_mod && y_mod){
    //         new_angle = new_angle + 180_stDeg;
    //     }
    //     else if(x_mod){
    //         new_angle = 180_stDeg - new_angle;
    //     }else if(y_mod){
    //         new_angle = -new_angle;
    //     }
    //
    //     return {x_mod ? -curr_pose.x : curr_pose.x,
    //             y_mod ? -curr_pose.y : curr_pose.y, new_angle};
    // }
    void setAngle(float theta){
        Robot::Globals::inertial_sensor.set_heading(theta);
        
        Robot::Globals::chassis.setPose(
            Robot::Globals::chassis.getPose().x,
            Robot::Globals::chassis.getPose().y,
            Robot::Globals::inertial_sensor.get_heading()
            );
    }
    void setPoint(Point point, Length std_deviation){
        Robot::Globals::particle_filter.init_normal_around_point(point, std_deviation);
        Robot::Globals::chassis.setPose(
            point.x.convert(in),
            point.y.convert(in),
            // doesnt change the theta by itself
            Robot::Globals::chassis.getPose().theta
            );
    }
    void setPose(Point point, float theta, Length std_deviation){
        setPoint(point,std_deviation);
        setAngle(theta);
    }
    void setPose(Length x, Length y,float theta, Length std_deviation){
        setPose({x,y},theta,std_deviation);
    }
}

