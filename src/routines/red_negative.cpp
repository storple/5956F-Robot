#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::RedNegative() {
	chassis.setPose(-24, -3, 180);

	subsystem.autonomous.ClampPID(40, 1500, true);
	pros::delay(250);

	chassis.moveToPoint(-24, 32, 1500);
	chassis.turnToHeading(-40, 1000, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPose(-66, 37.8, -90, 2750, {.lead=0.8, .maxSpeed=80, .minSpeed=55, });
	chassis.waitUntilDone();
	chassis.moveToPose(-24, 24, -55, 2000, {.forwards = false, .maxSpeed=100, .minSpeed=45});
	chassis.turnToPoint(-46, 20, 1000);
	chassis.moveToPoint(-46, 20, 1500, {.maxSpeed = 100});
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToPoint(0, 0, 750);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(0, -4, 1500, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(-8, 2, 1000, {.forwards=false});
}