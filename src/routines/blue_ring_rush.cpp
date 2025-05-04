#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;


// not real ring rush
void Autonomous::BlueRingRush() {

    chassis.setPose(61.2, 13, 135);

    ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(500);
	ArmMotor.move(-127);
	ArmMotor2.move(-127);
    
    chassis.moveToPoint(28, 24, 900, {.forwards=false, .maxSpeed=100}, false);
    ArmMotor.move(-20);
    ArmMotor2.move(-20);
	subsystem.pneumatics.toggleLatch();
	pros::delay(200);
	subsystem.intake.toggle();

    chassis.moveToPoint(8, 40, 1000, {});
	chassis.turnToPoint(12, 56, 750, {.maxSpeed=100}, false);
    chassis.moveToPoint(12, 56, 1500, {.maxSpeed=100});
    chassis.moveToPoint(25, 28, 1500, {.forwards=false});
	chassis.turnToHeading(0, 750, {.maxSpeed=100}, false);
	pros::delay(50);
	float rightDistance = distanceResetRightSensor();
	if (rightDistance != 9999) {
		chassis.setPose(rightDistance, chassis.getPose().y, chassis.getPose().theta);
	}
    chassis.moveToPoint(24, 48, 1100, {.maxSpeed=90}, false);
    left_motors.move(-127);
	right_motors.move(-127);
	pros::delay(300);

	chassis.moveToPoint(58, 58, 1300, {.maxSpeed=100});
	chassis.turnToHeading(45, 750, {.maxSpeed=100}, false);

	left_motors.move(70);
	right_motors.move(70);
	pros::delay(1500);
	left_motors.move(-80);
	right_motors.move(-80);
	pros::delay(800);
	left_motors.move(70);
	right_motors.move(70);
	pros::delay(1200);

    chassis.moveToPoint(48, 20, 1000);
    chassis.turnToPoint(48, 0, 750, {.maxSpeed=100}, false);
    subsystem.pneumatics.toggleIntakeLift();
    chassis.moveToPoint(48, 0, 1200, {.maxSpeed=80}, false);
    subsystem.pneumatics.toggleIntakeLift();
    left_motors.move(-70);
	right_motors.move(-70);
	pros::delay(300);
}