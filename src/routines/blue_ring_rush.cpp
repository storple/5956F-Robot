#include "lemlib/chassis/chassis.hpp"
#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;


// not real ring rush
void Autonomous::BlueRingRush() {
	localization::setPose(61.2_in, 13_in, 135, 2_in);

    ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(450);
	ArmMotor.brake();
	ArmMotor2.brake();

    chassis.moveToPoint(28, 24, 950, {.forwards=false, .maxSpeed=110}, false);
    
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	subsystem.arm.PID(250, 500);
    chassis.moveToPoint(8, 40, 1000, {}, false);
    chassis.moveToPoint(8, 56, 1500, {.maxSpeed=100}, false);
	chassis.turnToPoint(24, 20, 600, {.forwards=false, .maxSpeed=100}, false);
    chassis.moveToPoint(24, 20, 1500, {.forwards=false}, false);
	chassis.turnToPoint(24, 50, 600, {.maxSpeed=100}, false);
    chassis.moveToPoint(24, 50, 1300, {.maxSpeed=90}, false);
    
	chassis.moveToPoint(24, 36, 500, {.forwards=false}, false);

	chassis.moveToPoint(58, 58, 1200, {.maxSpeed=100}, false);
	chassis.turnToPoint(80, 80, 750, {.maxSpeed=100}, false);
	chassis.moveToPoint(80, 80, 1000, {.maxSpeed=100, .minSpeed=70}, false);
	chassis.moveToPoint(52, 52, 1400, {.forwards=false, .maxSpeed=80,.minSpeed=70}, false);
	chassis.moveToPoint(80, 80, 1000, {.maxSpeed=100,.minSpeed=70}, false);

    chassis.moveToPoint(48, 20, 1000, {}, false);
    chassis.turnToPoint(48, 0, 750, {.maxSpeed=100}, false);
    subsystem.pneumatics.toggleIntakeLift();
    chassis.moveToPoint(48, 0, 1200, {.maxSpeed=80}, false);
    subsystem.pneumatics.toggleIntakeLift();
    left_motors.move(-70);
	right_motors.move(-70);
	pros::delay(300);
}