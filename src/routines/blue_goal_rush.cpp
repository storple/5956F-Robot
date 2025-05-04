#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::BlueGoalRush() {
    chassis.setPose(53, -36.5, -105);

    ArmMotor.move(-20);
    ArmMotor2.move(-20);

    subsystem.intake.toggle();
    subsystem.pneumatics.toggleRightDoinker();
    chassis.moveToPoint(16, -48, 950, {.minSpeed=90});
    pros::delay(850);
    subsystem.intake.toggle();
    chassis.moveToPoint(40, -40, 1250, {.forwards=false, .minSpeed=70}, true);
    chassis.waitUntil(18);
    chassis.cancelAllMotions();
    pros::delay(400);
    chassis.turnToPoint(24, -24, 750, {.forwards=false, .minSpeed=80}, false);
    chassis.moveToPoint(24, -24, 900, {.forwards=false, .maxSpeed=90}, false);
    subsystem.pneumatics.toggleLatch();
    subsystem.pneumatics.toggleRightDoinker();
    pros::delay(150);
    subsystem.intake.toggle();
    pros::delay(400);
    chassis.turnToPoint(50, -26, 500);
    chassis.moveToPoint(50, -26, 1000, {.maxSpeed=100},false);
    pros::delay(400);
    chassis.moveToPose(40, -32, -135, 1000, {.maxSpeed=90}, false);
    subsystem.pneumatics.toggleLatch();
    moveMotors(30, 100);
    chassis.turnToPoint(12, -48, 750, {.forwards=false});
    chassis.moveToPoint(12, -48, 1400, {.forwards=false, .maxSpeed=90}, false);
    subsystem.pneumatics.toggleLatch();
    chassis.moveToPoint(38, -62, 650, {.maxSpeed=100});
    chassis.moveToPoint(56, -56, 900, {.maxSpeed=80});
    chassis.turnToHeading(135, 550, {}, false);
    
    left_motors.move(80);
	right_motors.move(80);
	subsystem.arm.PID(500, 600);
    pros::delay(500);
	left_motors.move(-80);
	right_motors.move(-80);
	pros::delay(600);
	left_motors.move(70);
	right_motors.move(70);
	pros::delay(1000);
    
    chassis.moveToPoint(54, -54, 1000, {.forwards = false});
    chassis.turnToHeading(-45,1000);
} 
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