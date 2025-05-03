#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "liblvgl/lvgl.h"

#include "localization/pose.h"
#include "localization/motion_model.h"
#include "localization/distance_model.h"
#include "localization/particle_filter.h"

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

pros::Motor LeftFront(-10, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor LeftMid(-8, pros::v5::MotorGears::blue,
                    pros::v5::MotorUnits::degrees);
pros::Motor LeftBack(9, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightFront(7, pros::v5::MotorGears::blue,
                       pros::v5::MotorUnits::degrees);
pros::Motor RightMid(6, pros::v5::MotorGears::blue,
                     pros::v5::MotorUnits::degrees);
pros::Motor RightBack(-5, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
pros::Motor IntakeMotor(14, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor ArmMotor(11, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor ArmMotor2(-13, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);

pros::MotorGroup left_motors({LeftFront.get_port(), LeftMid.get_port(),
                             LeftBack.get_port()}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({RightFront.get_port(), RightMid.get_port(),
                              RightBack.get_port()}, pros::MotorGearset::blue);

pros::adi::Pneumatics LeftDoinker('D', false);
pros::adi::Pneumatics RightDoinker('C', false);
pros::adi::Pneumatics LatchControl('A', false);
pros::adi::Pneumatics IntakeLift('B', false); 

pros::adi::DigitalIn LimitSwitch('F');

pros::Rotation arm_sensor(20); 
pros::Imu inertial_sensor(15);
pros::Optical color_sensor(20);
pros::Distance distance_sensor(20);

pros::Distance front_distance(17);
pros::Distance back_distance(19);
pros::Distance left_distance(16);
pros::Distance right_distance(18);

pros::Rotation horizontalEnc(-4);
pros::Rotation verticalEnc(-3);

// current offsets -2.625, 0.1675 
lemlib::TrackingWheel horizontal(&horizontalEnc, 1.995, -2.25); // figure the offsets
lemlib::TrackingWheel vertical(&verticalEnc, 1.995, -0.25);

RobotSubsystems subsystem;
AutonRoutes active_route = TEST;

bool playingRedSide = false;
bool useColorSort = false;  
bool useAutoClamp = true;
bool mogoGoalState = false;

lemlib::Drivetrain drivetrain(
    &left_motors, // left motor group
    &right_motors, // right motor group
    10.4375, // track width (inches)
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
    4,    // kP
    0,    // KI 
    3,    // kD
    0,    // Anti Windup
    0,    // smallErrorRange
    0,  // smallErrorTimeout
    0,    // largeErrorRange             
    0,  // largeErrorTimeout
    0    // slew rate
};

// turning PID
lemlib::ControllerSettings angular_controller{
    2.7,  // kP
    0,     // kI
    20, // kD
    0,     // Anti Windup
    0,     // smallErrorRange
    0,   // smallErrorTimeout
    0,     // largeErrorRange
    0,   // largeErrorTimeout
    0      // slew rate
};

/*
// forward/backward PID
lemlib::ControllerSettings lateral_controller{
    6,    // kP
    0,    // KI 
    8,    // kD
    0,    // Anti Windup
    0,    // smallErrorRange
    0,  // smallErrorTimeout
    0,    // largeErrorRange             
    0,  // largeErrorTimeout
    0    // slew rate
};

// turning PID
lemlib::ControllerSettings angular_controller{
    3,  // kP
    0,     // kI
    18, // kD
    0,     // Anti Windup
    0,     // smallErrorRange
    0,   // smallErrorTimeout
    0,     // largeErrorRange
    0,   // largeErrorTimeout
    0      // slew rate
};
*/

// lemlib::PID* current_arm_pid;

lemlib::PID current_arm_pid(
    0.8, // kP
    0, // kI
    0   , // kD
    5, // integral anti windup range
    false // don't reset integral when sign of error flips
);

// lemlib::PID arm_pid_score(
//     0.1, // kP
//     0, // kI
//     0   , // kD
//     5, // integral anti windup range
//     false // don't reset integral when sign of error flips
// );


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

localization::DistanceSensorModel front_distance_model(
    {1.4_in,4.1875_in}, // back offsets
    &front_distance,"front");

localization::DistanceSensorModel left_distance_model(
    {-0.25_in, 5.125_in}, // left offsets
    & left_distance,"left");

localization::DistanceSensorModel back_distance_model(
    {-4.75_in,-1.5_in}, // front offsets
    & back_distance,"back");

localization::DistanceSensorModel right_distance_model(
    {-4_in,-2.625_in}, // right offsets
    &right_distance,"right");

localization::MotionModel motion_model(
        &drivetrain,
        &verticalEnc, // vertical drift
        &horizontalEnc); // horizontal drift

localization::ParticleFilter<1000> particle_filter(&motion_model, 
    [](){
        // const Angle angle = -imu.get_heading() * deg;
        const Angle angle = from_cDeg(inertial_sensor.get_heading());
        return isfinite(angle.internal()) ? units::constrainAngle360_2(angle) : from_stDeg(infinity());
    });

}  // namespace Globals

}  // namespace Robot
