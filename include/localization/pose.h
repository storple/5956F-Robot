#pragma once

#include "units/units.hpp"
#include "units/Angle.hpp"
#include "math.h"

namespace localization {
    struct Point {
        Length x = 0.0_m;
        Length y = 0.0_m;
        constexpr Point operator+(const Point& other) const {
            return Point(x + other.x, y + other.y);
        }
        constexpr Point& operator+=(const Point& other){
            x += other.x;
            y += other.y;
            return (*this);
        }
    };
    struct Pose {
        Length x = 0.0_m;
        Length y = 0.0_m;
        Angle angle = 0_stDeg;
    };

    // returns (dx,dy)
    inline Point changeToPoint(const Point & p1,const Point & p2){
        return {p2.x - p1.x, p2.y - p1.y};
    }
    // returns (dx,dy,dtheta)
    inline Pose changeToPose(const Pose & p1,const Pose & p2){
        return {p2.x - p1.x, p2.y - p1.y, p2.angle-p1.angle};
    }

    inline Point pointFromPose(const Pose & pose){
	    return {pose.x, pose.y};
    }

    inline Pose poseFromPoint(const Point & point,Angle angle){
	    return {point.x, point.y, angle};
    }

    inline Point rotatePoint(const Point & point,const Angle& angle){
        const float sina = units::sin(angle).internal();
        const float cosa = units::cos(angle).internal();
        return {
            point.x * cosa - point.y * sina,
            point.y * cosa + point.x * sina
        };
    }
    inline Pose rotatePose(const Pose& pose,const Angle& angle){
        const float sina = units::sin(angle).internal();
        const float cosa = units::cos(angle).internal();

        return {
            pose.x * cosa - pose.y * sina,
            pose.y * cosa + pose.x * sina
        };
    }
}
