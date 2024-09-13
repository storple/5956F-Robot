#pragma once

#include "api.h"
#include "lemlib/api.hpp"

namespace Robot {
namespace Globals {
extern pros::Controller controller;

extern pros::Motor RightFront;
extern pros::Motor LeftFront;
extern pros::Motor LeftBack;
extern pros::Motor RightBack;
extern pros::Motor LeftMid;

extern pros::Motor RightMid;
extern pros::Motor IntakeMotor;
extern pros::Motor HookMotor;

extern pros::MotorGroup punchers;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup drive_;

extern pros::adi::DigitalIn drivetrainToggleSwitch;
extern pros::adi::DigitalIn autonToggleSwitch;

extern pros::adi::Pneumatics LatchControl;
extern pros::adi::Pneumatics ArmClaw;

extern lemlib::Drivetrain drivetrain;

extern lemlib::OdomSensors sensors;

// forward/backward PID
extern lemlib::ControllerSettings lateral_controller;

// turning PID
extern lemlib::ControllerSettings angular_controller;

// curvature
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::ExpoDriveCurve throttle_curve;

extern lemlib::Chassis chassis;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;



}  // namespace Globals
}  // namespace Robot