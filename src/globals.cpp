#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "liblvgl/lvgl.h"

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

pros::Motor LeftFront(-20, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor LeftMid(-17, pros::v5::MotorGears::blue,
                    pros::v5::MotorUnits::degrees);
pros::Motor LeftBack(-18, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightFront(16, pros::v5::MotorGears::blue,
                       pros::v5::MotorUnits::degrees);
pros::Motor RightMid(19, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightBack(12, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor IntakeMotor(-11, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor ArmMotor1(-9, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor ArmMotor2(10, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);


pros::MotorGroup left_motors({LeftFront.get_port(), LeftMid.get_port(),
                             LeftBack.get_port()}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({RightFront.get_port(), RightMid.get_port(),
                              RightBack.get_port()}, pros::MotorGearset::blue);

pros::adi::Pneumatics Doinker('A', false);
pros::adi::Pneumatics LatchControl('B', false);
pros::adi::Pneumatics IntakeLift('C', false); // TODO: port b will be IntakeLift, port C will be doinker, port D will be LimitSwitch

pros::adi::DigitalIn LimitSwitch('D');

pros::Rotation arm_sensor(7);
pros::Imu inertial_sensor(4);
pros::Optical color_sensor(15);
pros::Distance distance_sensor(13);
pros::Distance wall_sensor(14);
pros::Distance wall_sensor_back(8);

pros::Rotation horizontalEnc(-1);
pros::Rotation verticalEnc(-2);

// current offsets -2.625, 0.1675
lemlib::TrackingWheel horizontal(&horizontalEnc, 1.995, -2.9375); // figure the offsets
lemlib::TrackingWheel vertical(&verticalEnc, 1.995, 0.03125);

RobotSubsystems subsystem;
AutonRoutes active_route = BLUE_NEG;

bool playingRedSide = false;
bool useColorSort = false;
bool useAutoClamp = true;
bool mogoGoalState = false;

lemlib::Drivetrain drivetrain(
    &left_motors, // left motor group
    &right_motors, // right motor group
    11.3125, // track width (inches)
    lemlib::Omniwheel::NEW_325,
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
    6,    // kP
    0,    // KI 
    8,    // kD
    0,    // Anti Windup
    1,    // smallErrorRange
    100,  // smallErrorTimeout
    3,    // largeErrorRange             
    500,  // largeErrorTimeout
    0    // slew rate
};

// turning PID
lemlib::ControllerSettings angular_controller{
    3,  // kP
    0,     // kI
    18, // kD
    0,     // Anti Windup
    1,     // smallErrorRange
    100,   // smallErrorTimeout
    3,     // largeErrorRange
    500,   // largeErrorTimeout
    0      // slew rate
};

lemlib::PID arm_pid(
    0.01, // kP
    0, // kI
    0   , // kD
    5, // integral anti windup range
    false // don't reset integral when sign of error flips
);

lemlib::PID wall_sensor_pid(
    0.2, // kP
    0, // kI
    1 , // kD
    5, // integral anti windup range
    false // don't reset integral when sign of error flips
);

lemlib::PID distance_sensor_pid(
    0.34, // kP
    0, // kI
    1 , // kD
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