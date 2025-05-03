#pragma once

#include "pros/apix.h"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "globals.h"

#include <map>

// extern Angle angle_from_imu();
namespace localization{

    enum auton_corner_t {neg_blue, neg_red, pos_blue, pos_red};
 
    // inline auton_corner_t start_corner;
    inline auton_corner_t curr_corner;

    // void setStartCorner(auton_corner_t new_corner);
    void setCurrCorner(auton_corner_t new_corner);

    // float getNewAngle(float theta);
    // localization::Pose getNewPose(localization::Pose curr_pose);
    // localization::Point getNewPoint(localization::Point curr_point);

    //
    // only sets the point without changing angle
    void setPoint(Point point, Length std_deviation = 4_in);
    // theta must be within 0 - 360
    void setAngle(float theta);
    // set lemlib's pose around Point with angle in lemlib's format.
    // optionally can include the std_deviation as well
    void setPose(Point point, float theta, Length std_deviation = 4_in);
    void setPose(Length x, Length y, float theta, Length std_deviation = 4_in);
}

