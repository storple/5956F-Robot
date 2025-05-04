#include "globals.h"
#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

// estimated autos
// auto 1: sig solo
void Autonomous::BlueNegative() {
	chassis.setPose(61.2, 13, 135);

    ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(400);
	ArmMotor.move(-127);
	ArmMotor2.move(-127);
    
    chassis.moveToPoint(28, 24, 950, {.forwards=false, .maxSpeed=100}, false);
    ArmMotor.move(-20);
    ArmMotor2.move(-20);
	subsystem.pneumatics.toggleLatch();
	pros::delay(200);
	subsystem.intake.toggle();

    chassis.moveToPoint(10, 44, 1700, {.maxSpeed=100}, false);
    chassis.moveToPoint(28, 28, 1250, {.forwards=false});
	chassis.turnToHeading(0, 750, {.maxSpeed=100}, false);
	pros::delay(50);
	float rightDistance = distanceResetRightSensor();
	if (rightDistance != 9999) {
		chassis.setPose(rightDistance, chassis.getPose().y, chassis.getPose().theta);
	}
    chassis.moveToPoint(25, 48, 1100, {.maxSpeed=90}, false);
    left_motors.move(-127);
	right_motors.move(-127);
	pros::delay(300);

	chassis.moveToPoint(48, 0, 1750, {.maxSpeed=80});
	pros::delay(800);
	subsystem.pneumatics.toggleLatch();
	subsystem.pneumatics.toggleIntakeLift();
	pros::delay(650);
	subsystem.pneumatics.toggleIntakeLift();
	chassis.cancelAllMotions();
	pros::delay(400);
	subsystem.intake.toggle();
	moveMotors(-60, 300);
	chassis.moveToPoint(48, 0, 600, {.forwards=false, .maxSpeed=120}, false);
	chassis.turnToPoint(23, -22, 750, {.forwards=false, .maxSpeed=100}, false);
	chassis.moveToPoint(23, -22, 1200, {.forwards=false, .maxSpeed=100}, false);
	subsystem.pneumatics.toggleLatch();
	chassis.turnToPoint(24, -48, 700, {.maxSpeed=100});
	pros::delay(200);
	subsystem.intake.toggle();
	chassis.moveToPoint(24, -48, 1500, {.maxSpeed=100});
	pros::delay(500);
	subsystem.arm.PID(350, 450);
	chassis.waitUntilDone();
	chassis.turnToPoint(0, 0, 600, {.maxSpeed=100}, false);
	left_motors.move(70);
	right_motors.move(70);


}

/*
// auto 2
void Autonomous::BlueNegative() {
	chassis.setPose(60, 24, -70);
	subsystem.pneumatics.toggleRightDoinker();
	subsystem.intake.toggle();
	chassis.moveToPoint(8, 42, 1000, {.minSpeed=100}, false);
	chassis.turnToPoint(24, 24, 500, {.forwards=false}, false);
	chassis.moveToPoint(22, 26, 750, {.forwards=false, .maxSpeed=100}, false);
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(200);
	subsystem.intake.toggle();
	chassis.turnToHeading(-20, 300);
	pros::delay(100);
	subsystem.pneumatics.toggleRightDoinker();
	chassis.moveToPose(30, 52, 45, 1000, {.lead=0.6}, false);
	chassis.turnToHeading(90, 200);
	chassis.moveToPose(58, 54, 45, 750, {}, false);
	
	// corner ring
	left_motors.move(60);
	right_motors.move(60);
	pros::delay(500);
	// i have no idea how this intake works 
	chassis.moveToPoint(58,54, 500, {.forwards=false});
	chassis.turnToPoint(50, 25, 300);
	chassis.moveToPoint(50, 22, 1000);
	

}

// auto 3
void Autonomous::BlueNegative() {
	chassis.setPose(60, 12, 135);

	ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(500);
	ArmMotor.brake();
	ArmMotor2.brake();

	chassis.moveToPoint(30, 24, 750, {.forwards=false}, false);
	subsystem.pneumatics.toggleLatch();
	pros::delay(50);
	chassis.turnToHeading(-45, 500);
	subsystem.intake.toggle();
	chassis.moveToPose(5, 52, 0, 1500, {.maxSpeed=80}, false);
	chassis.moveToPose(12, 30, -30, 1000, {.forwards=false, .minSpeed=80});
	chassis.moveToPoint(24, 48, 1000, {}, false);
	chassis.moveToPoint(58, 58, 1000);
	chassis.turnToHeading(45, 750, {}, false);

	left_motors.move(80);
	right_motors.move(80);
	pros::delay(500);
	left_motors.move(-100);
	right_motors.move(-100);
	pros::delay(300);
	left_motors.move(80);
	right_motors.move(80);
	pros::delay(200);
	
	chassis.moveToPoint(48, 24, 1000, {.forwards=false}, false);
	chassis.turnToPoint(48, 0, 750);
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(48, 2, 750,{.maxSpeed=60}, false);
	left_motors.move(-60);
	right_motors.move(-60);
	pros::delay(300);
	chassis.turnToHeading(-15, 750, {}, false);
	left_motors.move(-80);
	right_motors.move(-80);

}

*/
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

// void Autonomous::BlueNegative() {
// 	chassis.setPose(24, -3, -180);

// 	subsystem.autonomous.ClampPID(40, 1500, true);
// 	pros::delay(250);

// 	chassis.moveToPoint(24, 32, 1500);
// 	chassis.turnToHeading(40, 1000, {.maxSpeed=60});
// 	chassis.waitUntilDone();
// 	subsystem.intake.toggle();
// 	chassis.moveToPose(66, 37.8, 90, 2750, {.lead=0.8, .maxSpeed=80, .minSpeed=55, });
// 	chassis.waitUntilDone();
// 	chassis.moveToPose(24, 24, 55, 2000, {.forwards = false, .maxSpeed=100, .minSpeed=45});
// 	chassis.turnToPoint(46, 20, 1000);
// 	chassis.moveToPoint(46, 20, 1500, {.maxSpeed = 100});
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	chassis.turnToPoint(0, 0, 750);
// 	chassis.waitUntilDone();
// 	subsystem.pneumatics.toggleIntakeLift();
// 	chassis.moveToPoint(0, -4, 1500, {.maxSpeed=60});
// 	chassis.waitUntilDone();
// 	subsystem.pneumatics.toggleIntakeLift();
// 	chassis.moveToPoint(8, 2, 1000, {.forwards=false});
// }