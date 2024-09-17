#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include "robot/auton.h"
#include "robot/arm.h"
#include "robot/drivetrain.h"
#include "robot/intake.h"
#include "robot/latch.h"

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
extern pros::Motor ArmMotor;

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

extern pros::Imu arm_sensor;
extern pros::Imu inertial_sensor;

/**
 * @brief Structure that holds instances of all robot subsystems.
 */
struct RobotSubsystems {
  Robot::Autonomous autonomous;
  Robot::Drivetrain drivetrain;
  Robot::Intake intake;
  Robot::Latch latch;
  Robot::Arm arm;
};

extern RobotSubsystems subsystem;

// forward/backward PID
extern lemlib::ControllerSettings lateral_controller;

// turning PID
extern lemlib::ControllerSettings angular_controller;

extern lemlib::PID arm_pid;

// curvature
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::ExpoDriveCurve throttle_curve;

extern lemlib::Chassis chassis;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;



}  // namespace Globals
}  // namespace Robot