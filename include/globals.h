#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include "robot/auton.h"
#include "robot/arm.h"
#include "robot/drivetrain.h"
#include "robot/intake.h"
#include "robot/pneumatics.h"
#include "robot/screen.h"

namespace Robot {
namespace Globals {
extern pros::Controller controller;

extern pros::Motor LeftFront;
extern pros::Motor LeftMid;
extern pros::Motor LeftBack;
extern pros::Motor RightFront;
extern pros::Motor RightMid;
extern pros::Motor RightBack;
extern pros::Motor IntakeMotor;
extern pros::Motor ArmMotor1;
extern pros::Motor ArmMotor2;

extern pros::MotorGroup punchers;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup drive_;

extern pros::adi::Pneumatics LatchControl;
extern pros::adi::Pneumatics IntakeLift;
extern pros::adi::Pneumatics Doinker;
extern pros::adi::Pneumatics Doinker2;

extern pros::adi::DigitalIn LimitSwitch;

extern lemlib::Drivetrain drivetrain;

extern lemlib::OdomSensors sensors;

extern pros::Rotation arm_sensor;
extern pros::Imu inertial_sensor;
extern pros::Optical color_sensor;
extern pros::Distance distance_sensor;
extern pros::Distance wall_sensor;
extern pros::Distance wall_sensor_back;
extern pros::Distance wall_sensor_side;

/**
 * @brief Structure that holds instances of all robot subsystems.
 */
struct RobotSubsystems {
  Robot::Autonomous autonomous;
  Robot::Drivetrain drivetrain;
  Robot::Intake intake;
  Robot::Pneumatics pneumatics;
  Robot::Arm arm;
};

extern RobotSubsystems subsystem;

enum AutonRoutes { RED_NEG, RED_POS, RED_GOALRUSH, RED_RING, BLUE_NEG, BLUE_POS, BLUE_GOALRUSH, BLUE_RING, SKILLS, NONE, TEST };
extern AutonRoutes active_route;

extern bool playingRedSide;
extern bool useColorSort;
extern bool useAutoClamp;
extern bool mogoGoalState;

// forward/backward PID
extern lemlib::ControllerSettings lateral_controller;

// turning PID
extern lemlib::ControllerSettings angular_controller;

extern lemlib::PID arm_pid;
extern lemlib::PID wall_sensor_pid;
extern lemlib::PID distance_sensor_pid;

// curvature
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::ExpoDriveCurve throttle_curve;

extern lemlib::Chassis chassis;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;



}  // namespace Globals
}  // namespace Robot