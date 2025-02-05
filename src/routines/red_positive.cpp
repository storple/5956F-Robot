#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::RedPositive() {
	chassis.setPose(12.5, -12, -135);
	subsystem.arm.PID(15400, 500);
	chassis.moveToPoint(24, 0, 1500, {.maxSpeed=50});
	chassis.waitUntilDone();
	pros::delay(300);
	subsystem.arm.PID(1400, 500);
	chassis.turnToPoint(24, 24, 1500, {.forwards=false, .maxSpeed=50});

	//backwards autoclamp
	ClampPID(40, 2500, true);
	// subsystem.pneumatics.toggleLatch();

	chassis.turnToPoint(48, 24, 750, {.maxSpeed=70});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPoint(44, 24, 1500, {.maxSpeed=70});
	chassis.waitUntilDone();
	pros::delay(750);

	chassis.turnToPoint(34, 0, 750, {.maxSpeed=60});
	chassis.moveToPoint(34, 0, 1500, {.maxSpeed=60});
	chassis.turnToPoint(0, 0, 750, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(6, 2, 1500, {.maxSpeed=60}, false);
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(12, 0, 1000, {.forwards=false, .maxSpeed=70});
	chassis.turnToPoint(0, 24, 750, {.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, 24, 3000, {.maxSpeed=50});
}