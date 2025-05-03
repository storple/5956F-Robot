#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::BlueGoalRush() {
    chassis.setPose(-36, -1.5, -16);

    ArmMotor.move(-20);

    subsystem.intake.toggle();
    subsystem.pneumatics.toggleRightDoinker();
    chassis.moveToPoint(-47, 30, 950, {.minSpeed=80});
    pros::delay(750);
    subsystem.pneumatics.toggleRightDoinker();
    pros::delay(100);
    subsystem.intake.toggle();
    chassis.moveToPoint(-52, 18, 1250, {.forwards=false, .minSpeed=60}, true);
    chassis.waitUntil(14);
    subsystem.pneumatics.toggleRightDoinker();
    chassis.cancelAllMotions();
    pros::delay(750);
    chassis.turnToPoint(-24, 22, 1000, {.forwards=false, .maxSpeed=90}, false);
    ClampPID(10, 1750, false);
    subsystem.pneumatics.toggleRightDoinker();
    pros::delay(250);
    subsystem.intake.toggle();
    chassis.turnToPoint(-24, -4, 500);
    chassis.moveToPoint(-24, -4, 1250, {.maxSpeed=70});
    chassis.moveToPose(-30, -8, -45, 1000, {.maxSpeed=90}, false);
    subsystem.pneumatics.toggleLatch();
    moveMotors(30, 100);
    chassis.turnToPoint(-36, 24, 750, {.forwards=false});
    moveMotors(-40, 500);
    chassis.moveToPoint(-42, 36, 1500, {.forwards=false, .maxSpeed=80}, false);
    subsystem.pneumatics.toggleLatch();
    chassis.turnToPoint(-68, -16, 750, {.maxSpeed=80}, false);
    moveMotors(80, 2250);
    moveMotors(-40, 500);
    moveMotors(40, 400);
    moveMotors(-40, 300);

    chassis.moveToPoint(-48, 36, 1500, {.forwards = false, .maxSpeed=80});
    
    // if (distance_sensor.get() < 1200) {
    //     ClampPID(10, 1750, false);
    // }
   /* subsystem.intake.toggle();
    pros::delay(250);
    subsystem.intake.toggle();
    subsystem.pneumatics.toggleIntakeLift();
    chassis.turnToPoint(0, 0, 500);
    chassis.moveToPoint(0, 0, 2000, {.maxSpeed=60});

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
    chassis.moveToPoint(-40, 24, 1000, {.forwards=false, .maxSpeed=50}, false);
    */
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