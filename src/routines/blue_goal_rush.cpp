#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::BlueGoalRush() {
    chassis.setPose(-58.5, -2.1, 15);
    subsystem.intake.toggle();
    subsystem.pneumatics.toggleDoinker();
    chassis.moveToPoint(-49, 30, 950);
    pros::delay(850);
    subsystem.intake.toggle();
    chassis.waitUntil(30);
    subsystem.pneumatics.toggleDoinker();
    chassis.moveToPoint(-52, 18, 1250, {.forwards=false, .minSpeed=60}, true);
    chassis.waitUntil(14);
    subsystem.pneumatics.toggleDoinker();
    chassis.cancelAllMotions();
    pros::delay(750);
    chassis.turnToPoint(-24, 24, 1000, {.forwards=false, .maxSpeed=90}, false);
    ClampPID(10, 1750, false);
    subsystem.pneumatics.toggleDoinker();
    pros::delay(250);
    subsystem.intake.toggle();
    subsystem.pneumatics.toggleIntakeLift();
    chassis.turnToPoint(0, 0, 500);
    chassis.moveToPoint(0, 0, 2500, {.maxSpeed=60});
    chassis.waitUntil(24);
    subsystem.pneumatics.toggleIntakeLift();
    chassis.moveToPoint(-12, 0, 1000, {.forwards=false, .maxSpeed=50}, false);
    chassis.turnToPoint(-60, -16, 1000, {.maxSpeed=80});
    chassis.moveToPoint(-60, -16, 1500, {.maxSpeed=80});
    pros::delay(700);
    subsystem.pneumatics.toggleLatch();
    LimitSwitchIntake(650);
    chassis.turnToPoint(-50, 24, 750, {.forwards=false}, false);
    chassis.moveToPoint(-55, 8, 750, {.forwards=false, .maxSpeed=70}, false);
    if (distance_sensor.get() < 1600) {
        ClampPID(10, 1750, false);
        pros::delay(250);
        subsystem.intake.toggle();
    }
}

/*
void Autonomous::BlueGoalRush() {
    int m = 1; // -1: red, 1: blue ||| RED SIDE: FIRST SLOT, BLUE: SECOND SLOT
	chassis.setPose(-24*m, -4, 206);
	chassis.moveToPoint(-45*m, 39, 1500, {.forwards=false}, false);
	subsystem.pneumatics.toggleLatch();
	pros::delay(250);
	subsystem.intake.toggle();
	chassis.moveToPoint(-48*m, 24, 1500);
	chassis.turnToPoint(-60*m, 5, 1000, {.forwards=false});
	chassis.moveToPoint(-60*m, 5, 2500, {.forwards=false}, false);
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(250);
	chassis.turnToPoint(-24*m, -6, 1000);
	chassis.moveToPoint(-24*m, -6, 1000);
	chassis.turnToPoint(-24*m, 24, 1000, {.forwards=false});
	chassis.waitUntilDone();
	ClampPID(40, 2500, true);
	subsystem.intake.toggle();

	chassis.moveToPoint(-55*m, 12, 3000);
}
*/  