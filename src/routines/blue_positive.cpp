#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::BluePositive() {
	chassis.setPose(61.2, -13, 45);   //61.2, -13,


	ArmMotor.move(127);
	ArmMotor2.move(127);
	pros::delay(500);
	ArmMotor.brake();
	ArmMotor2.brake();
	subsystem.arm.PID(0, 500);
	ArmMotor.move(-20);
	ArmMotor2.move(-20);

	chassis.moveToPoint(28, -24, 1200, {.forwards=false, .maxSpeed=100}, false);
	subsystem.pneumatics.toggleLatch();
	pros::delay(200);
	subsystem.intake.toggle();
	chassis.turnToPoint(24, -46, 1000, {.maxSpeed=90});

	chassis.moveToPoint(24, -46, 950, {.maxSpeed=70}, false);
	left_motors.move(-70);
	right_motors.move(-70);
	pros::delay(300);

	chassis.moveToPoint(49, -58, 1300, {.maxSpeed=100});
	chassis.moveToPoint(56, -48, 1300, {.maxSpeed=80}, false);
	chassis.turnToHeading(135, 750, {.maxSpeed=90}, false);

	left_motors.move(70);
	right_motors.move(70);
	pros::delay(1300);
	left_motors.move(-70);
	right_motors.move(-70);
	pros::delay(750);
	left_motors.move(70);
	right_motors.move(70);
	pros::delay(1000);

	chassis.moveToPoint(56, -56, 1000, {.forwards=false}, false);
	chassis.turnToPoint(0, 0, 750, {.maxSpeed=100}, false);
	left_motors.move(50);
	right_motors.move(50);
	subsystem.arm.PID(350, 1000);

	
}

/////////////////////////
/////////////////////////
/////////////////////////
/////////////////////////
/////////////////////////

// void Autonomous::BluePositive() {
// 	chassis.setPose(-12.5, -12, 135);
// 	subsystem.arm.PID(15400, 500);
// 	chassis.moveToPoint(-24, 0, 1500, {.maxSpeed=50});
// 	chassis.waitUntilDone();
// 	pros::delay(300);

// 	subsystem.arm.PID(500, 500);
// 	ArmMotor.move(-20);

// 	chassis.turnToPoint(-24, 24, 1500, {.forwards=false, .maxSpeed=50});

// 	//backwards autoclamp
// 	ClampPID(40, 2500, true);
// 	// subsystem.pneumatics.toggleLatch();

// 	chassis.turnToPoint(-48, 24, 750, {.maxSpeed=70});
// 	chassis.waitUntilDone();
// 	subsystem.intake.toggle();
// 	chassis.moveToPoint(-46.5, 24, 1500, {.maxSpeed=70});
// 	chassis.waitUntilDone();
// 	pros::delay(750);

// 	chassis.turnToPoint(-34, 0, 750, {.maxSpeed=60});
// 	chassis.moveToPoint(-34, 0, 1500, {.maxSpeed=60});
// 	chassis.turnToPoint(0, 0, 750, {.maxSpeed=60});
// 	chassis.waitUntilDone();
// 	subsystem.pneumatics.toggleIntakeLift();
// 	chassis.moveToPoint(-6, 0, 1500, {.maxSpeed=60}, false);
// 	subsystem.pneumatics.toggleIntakeLift();
// 	chassis.moveToPoint(-12, 0, 1000, {.forwards=false, .maxSpeed=70});
// 	chassis.turnToPoint(0, 24, 750, {.maxSpeed=90});
// 	chassis.waitUntilDone();
// 	chassis.moveToPoint(0, 24, 3000, {.maxSpeed=50});
// }