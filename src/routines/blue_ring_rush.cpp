#include "lemlib/chassis/chassis.hpp"
#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;


// not real ring rush
void Autonomous::BlueRingRush() {
	// localization::setPose(58_in, 16_in, 135, 3_in);
	chassis.setPose(58, 16, 135);

    ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(500);
	ArmMotor.move(-127);
	ArmMotor2.move(-127);

    chassis.moveToPoint(28, 24, 900, {.forwards=false, .maxSpeed=100}, false);
    ArmMotor.move(-20);
    ArmMotor2.move(-20);
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	
    chassis.moveToPoint(8, 40, 1000, {}, false);
    chassis.moveToPoint(8, 56, 1500, {.maxSpeed=100}, false);
    chassis.moveToPoint(24, 28, 1500, {.forwards=false}, false);
    chassis.moveToPoint(24, 50, 1300, {.maxSpeed=90}, false);
    left_motors.move(-127);
	right_motors.move(-127);
	pros::delay(300);

	chassis.moveToPoint(58, 58, 1200, {.maxSpeed=100}, false);
	chassis.turnToPoint(66, 66, 1000, {.maxSpeed=100}, false);
	chassis.moveToPoint(58, 58, 1000, {.forwards=false, .maxSpeed=80}, false);
	chassis.moveToPoint(66, 66, 1000, {.maxSpeed=80}, false);

    chassis.moveToPoint(48, 20, 1000, {}, false);
    chassis.turnToPoint(48, 0, 750, {.maxSpeed=100}, false);
    subsystem.pneumatics.toggleIntakeLift();
    chassis.moveToPoint(48, 0, 1200, {.maxSpeed=80}, false);
    subsystem.pneumatics.toggleIntakeLift();
    left_motors.move(-70);
	right_motors.move(-70);
	pros::delay(300);
}