#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::RedRingRush() {
    chassis.setPose(-30, -1.5, -16);
    subsystem.intake.toggle();
    subsystem.pneumatics.toggleDoinker();
    chassis.moveToPoint(-40, 33, 950, {.minSpeed=100});
    chassis.waitUntil(33);
    chassis.cancelAllMotions();
    chassis.moveToPoint(-38, 32, 1250, {.forwards=false, .minSpeed=60}, true);
    pros::delay(300);
    subsystem.intake.toggle();
    pros::delay(450);
    chassis.turnToPoint(-24, 22, 750, {.forwards=false, .maxSpeed=90}, false);
    ClampPID(10, 1750, false);
    pros::delay(250);
    subsystem.pneumatics.toggleDoinker();
    subsystem.intake.toggle();
    chassis.moveToPoint(-48, 24, 1250, {.maxSpeed=70});
    chassis.turnToPoint (-24,0,750);
    chassis.moveToPose(-22, -4, 45, 1000, {.maxSpeed=90});
    chassis.turnToPoint(0, 0, 500);
    subsystem.pneumatics.toggleIntakeLift();
    chassis.moveToPoint(2, 0, 2000, {.maxSpeed=60});

    chassis.moveToPoint(-30, 0, 1000, {.forwards=false, .maxSpeed=50}, false);
    int start_time = pros::millis();

	while (true) {
		if (LimitSwitch.get_new_press()) {
			break;
		}
		pros::delay(15);
    }
    pros::delay(400);
	subsystem.intake.toggle();
    subsystem.pneumatics.toggleIntakeLift();
}