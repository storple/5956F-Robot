#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"

/*
 * Although the following constants belong in their own seperate
 * files(auton.cpp, drivetrain.cpp), they are put here in order to maintain a
 * common location for all of the constants used by the program to belong in.
 * NOTE: This is the location where these variables are put into memory, but
 * they can be otherwise modified throughout the program.
 */

// Defines the objects that are used by the program for each of the individual
// subsystems.

namespace Robot {
namespace Globals {

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor LeftFront(2, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor LeftMid(12, pros::v5::MotorGears::blue,
                    pros::v5::MotorUnits::degrees);
pros::Motor LeftBack(11, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightFront(10, pros::v5::MotorGears::blue,
                       pros::v5::MotorUnits::degrees);
pros::Motor RightMid(19, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightBack(20, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor IntakeMotor(9, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor ArmMotor(14, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);

pros::MotorGroup left_motors({LeftFront.get_port(), LeftMid.get_port(),
                             LeftBack.get_port()});
pros::MotorGroup right_motors({RightFront.get_port(), RightMid.get_port(),
                              RightBack.get_port()});

pros::adi::DigitalIn drivetrainToggleSwitch('G');
pros::adi::DigitalIn autonToggleSwitch('F');

pros::adi::Pneumatics LatchControl('A', false);
pros::adi::Pneumatics ArmClaw('C', false);

pros::Imu arm_sensor(15);
pros::Imu inertial_sensor(17);

pros::Rotation horizontalEnc(13);
pros::Rotation verticalEnc(18);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75); // figure the offsets
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

RobotSubsystems subsystem;

// Describes the lemlib objects that are used to control the autonomous
// functions of the robot.
// TODO: adjust settings.
lemlib::Drivetrain drivetrain(
    &left_motors, // left motor group
    &right_motors, // right motor group
    13.45, // track width (inches)
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450, // drivetrain rpm 
    2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(
    &vertical,   // vertical tracking wheel
    nullptr,     // vertical tracking wheel 2, set to nullptr as we don't have a second one
    &horizontal, // horizontal tracking wheel
    nullptr,     // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &inertial_sensor        // inertial sensor
);

// forward/backward PID
lemlib::ControllerSettings lateral_controller{
    1,  // kP
    0,    // KI
    0,    // kD
    3,    // Anti Windup
    1,    // smallErrorRange
    100,  // smallErrorTimeout
    3,    // largeErrorRange
    500,  // largeErrorTimeout
    90    // slew rate
};

// turning PID
lemlib::ControllerSettings angular_controller{
    1,  // kP
    0,     // kI
    0,  // kD
    3,     // Anti Windup
    1,     // smallErrorRange
    100,   // smallErrorTimeout
    3,     // largeErrorRange
    500,   // largeErrorTimeout
    0      // slew rate
};

lemlib::PID arm_pid(
    1, // kP
    0, // kI
    0, // kD
    5, // integral anti windup range
    false // don't reset integral when sign of error flips
);

lemlib::ExpoDriveCurve throttle_curve(
    3,     // joystick deadband out of 127
    10,    // minimum output where drivetrain will move out of 127
    1.019  // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
    4,      // joystick deadband out of 127
    6,      // minimum output where drivetrain will move out of 127
    1.019   // expo curve gain
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

}  // namespace Globals

}  // namespace Robot