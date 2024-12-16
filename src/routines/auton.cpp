#include "robot/auton.h"
#include "main.h" // IWYU pragma: export

using namespace Robot;
using namespace Robot::Globals;

ASSET(abc_txt);

void Autonomous::WallSensorPID(float target_distance, int timeout, bool back) {

	if (!back) {
    int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (wall_sensor.get());

        double output = wall_sensor_pid.update((double) error);

        left_motors.move(-output);
		right_motors.move(-output);

        if ((fabs(error) < 5) or (pros::millis() - start_time > timeout)) { 
            left_motors.brake();
			right_motors.brake();
            break;
        }
        
        pros::delay(20);
    }
	}

	else {
		
    int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (wall_sensor_back.get());

        double output = wall_sensor_pid.update((double) error);

        left_motors.move(output);
		right_motors.move(output);

        if ((fabs(error) < 5) or (pros::millis() - start_time > timeout)) { 
            left_motors.brake();
			right_motors.brake();
            break;
        }
        
        pros::delay(20);
    }
	}
}

void Autonomous::RedNegative() {
	chassis.setPose(0, 0, 0);

	//backwards autoclamp
	left_motors.move(-60);
	right_motors.move(-60);
	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(135, 1000);
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPose(32, -36, 90, 3000, {.maxSpeed = 65});
	pros::delay(500);
	chassis.moveToPose(0, -18, 120, 2000, {.forwards = false, .maxSpeed=127});
	chassis.turnToHeading(90, 1000);
	chassis.moveToPoint(24, -18, 2000, {.maxSpeed = 100});
	chassis.turnToHeading(-30, 1000);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(-26, 4, 2500, {.maxSpeed=60});
	chassis.moveToPoint(-22, 4, 1000, {.forwards=false});
}
void Autonomous::RedPositive() {
	chassis.setPose(0, 0, 38);
	chassis.moveToPoint(0, 5, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(15400);
	chassis.turnToHeading(15, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(1400);

	//backwards autoclamp
	left_motors.move(-60);
	right_motors.move(-60);
	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(-90, 1000);
	chassis.waitUntilDone();

	subsystem.intake.toggle();
	chassis.moveToPoint(-28, -30, 2500, {.maxSpeed=80});
	chassis.waitUntilDone();
	chassis.turnToHeading(60, 1000);
	subsystem.pneumatics.toggleIntakeLift();

	chassis.moveToPoint(14, -4, 3000, {.maxSpeed=60});
	chassis.moveToPoint(10, -4, 1000);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.turnToHeading(-180, 1000);
	chassis.moveToPoint(18, -30, 2000, {.maxSpeed=40});
}
void Autonomous::BlueNegative() {
	chassis.setPose(0, 0, 0);

	//backwards autoclamp
	left_motors.move(-60);
	right_motors.move(-60);
	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(-135, 1000);
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPose(-32, -36, -90, 3000, {.maxSpeed = 65});
	pros::delay(500);
	chassis.moveToPose(0, -18, -120, 2000, {.forwards = false, .maxSpeed=127});
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPoint(-24, -18, 2000, {.maxSpeed = 100});
	chassis.turnToHeading(30, 1000);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(26, 4, 2500, {.maxSpeed=60});
	chassis.moveToPoint(22, 4, 1000, {.forwards=false});
}

void Autonomous::BluePositive() {
	chassis.setPose(0, 0, -38);
	chassis.moveToPoint(0, 5, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(15400);
	chassis.turnToHeading(-15, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(1400);

	//backwards autoclamp
	left_motors.move(-60);
	right_motors.move(-60);
	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(90, 1000);
	chassis.waitUntilDone();

	subsystem.intake.toggle();
	chassis.moveToPoint(28, -30, 2500, {.maxSpeed=80});
	chassis.waitUntilDone();
	chassis.turnToHeading(-60, 1000);
	subsystem.pneumatics.toggleIntakeLift();

	chassis.moveToPoint(-14, -4, 3000, {.maxSpeed=60});
	chassis.moveToPoint(-10, -4, 1000);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(-18, -30, 2000, {.maxSpeed=40});
}

// Skills
void Autonomous::Skills() 
{
	chassis.setPose(0, 0, 0);

// subsystem.autonomous.WallSensorPID(1000, 2000);
	// chassis.turnToHeading(190, 5000);
	// pros::delay(5000);
	// chassis.turnToHeading(240, 5000);

	// subsystem.autonomous.WallSensorPID(1000, 3000, false);
	// pros::delay(100000);

	subsystem.intake.toggle();
	pros::delay(500);
	chassis.turnToHeading(100, 1000);
	chassis.waitUntilDone();

	// go backwards to mogo
	left_motors.move(-60);
	right_motors.move(-60);

	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}

	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(750);
	subsystem.intake.toggle();

	chassis.moveToPoint(-20, 26, 1000);
	chassis.turnToHeading(-50, 750);
	chassis.moveToPoint(-44, 76, 2000);
	chassis.waitUntilDone();
	chassis.turnToHeading(210, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(4000);
	subsystem.autonomous.WallSensorPID(1615, 1750, false);
	subsystem.arm.PID(4000);
	// chassis.moveToPoint(-60, 54, 1500);
	// chassis.waitUntilDone();

	chassis.turnToHeading(-90, 750);
	chassis.waitUntilDone();

	// chassis.moveToPoint(-68, 64, 1000);
	// chassis.waitUntilDone();
	
	subsystem.autonomous.WallSensorPID(60, 500, false);
		
	pros::delay(500);
	subsystem.intake.toggle();
	pros::delay(700);
	subsystem.arm.PID(15400);


	subsystem.autonomous.WallSensorPID(450, 500, false);

	// chassis.moveToPoint(-44, 60, 1000, {.forwards=false});
	// chassis.waitUntilDone();

	subsystem.intake.toggle();
	subsystem.arm.PID(1400);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(-44, 20, 2000);
	chassis.moveToPoint(-44, -5, 2500, {.maxSpeed=60});
	chassis.moveToPoint(-58, 6, 2500);
	chassis.waitUntilDone();
	chassis.moveToPoint(-62, -10, 2000, {.forwards=false});
	subsystem.intake.toggle();
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleLatch();
	

	chassis.moveToPoint(-24, 0, 2000, {.maxSpeed=80});
	chassis.turnToHeading(90, 750);
	chassis.waitUntilDone();
	subsystem.autonomous.WallSensorPID(1650, 2000, false);
	chassis.setPose(0, 0, 90);
	chassis.turnToHeading(-90, 750);
	chassis.waitUntilDone();
	subsystem.intake.toggle();

	/////////////////////
	/////////////////////

	left_motors.move(-60);
	right_motors.move(-60);

	start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}

	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();
	// subsystem.intake.toggle();
	pros::delay(750);
	subsystem.intake.toggle();

	chassis.moveToPoint(23, 20, 1000);
	chassis.turnToHeading(45, 750);
	chassis.moveToPoint(39, 76, 2000);
	chassis.waitUntilDone();
	chassis.turnToHeading(-210, 750);
	chassis.waitUntilDone();
	subsystem.arm.PID(4000);
	subsystem.autonomous.WallSensorPID(1860, 2000, true);
	subsystem.arm.PID(4000);
	// chassis.moveToPoint(-60, 54, 1500);
	// chassis.waitUntilDone();

	chassis.turnToHeading(90, 750);
	chassis.waitUntilDone();

	// chassis.moveToPoint(-68, 64, 1000);
	// chassis.waitUntilDone();
	
	subsystem.autonomous.WallSensorPID(60, 500, false);
		
	pros::delay(500);
	subsystem.intake.toggle();
	pros::delay(700);
	subsystem.arm.PID(15400);


	subsystem.autonomous.WallSensorPID(450, 750, false);

	// chassis.moveToPoint(-44, 60, 1000, {.forwards=false});
	// chassis.waitUntilDone();

	subsystem.intake.toggle();
	subsystem.arm.PID(1400);
	chassis.turnToHeading(-180, 750);
	chassis.moveToPoint(44, 25, 1000);
	chassis.moveToPoint(44, -10, 2000, {.maxSpeed=60});
	chassis.moveToPoint(52, 2, 2500);
	chassis.waitUntilDone();
	chassis.moveToPoint(56, -10, 2000, {.forwards=false});
	subsystem.intake.toggle();
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(0, 1500);
	chassis.moveToPoint(48, 54, 1500);
	chassis.moveToPoint(-24, 105, 1500);
	chassis.moveToPoint(-72, 115, 1500);
	chassis.moveToPoint(72, 115, 1500);

	// chassis.moveToPose(24, 96, -30, 3000);

	// start_time = pros::millis();
	// while (!subsystem.intake.detectRing()) { 
	// 	pros::delay(20);
	// 	if (pros::millis() - start_time > 2200) break; // failsafe
	// }
	// subsystem.intake.toggle();

	// chassis.turnToHeading(135, 3000);
	
	// left_motors.move(-60);
	// right_motors.move(-60);

	// start_time = pros::millis();
	// while (!subsystem.pneumatics.detectMogo()) {
	// 	pros::delay(60);
	// 	if (pros::millis() - start_time > 1500) break;
	// }
	// left_motors.brake();
	// right_motors.brake();
	// subsystem.pneumatics.toggleLatch();
	// subsystem.intake.toggle();

	// chassis.turnToHeading(-135, 3000);
	// chassis.moveToPoint(-24, 96, 3000);
	
	// chassis.turnToHeading(-30, 3000);
	// chassis.moveToPoint(-36, 120, 300);
	// chassis.moveToPoint(-66, 120, 1000);
	// chassis.moveToPoint(-48, 132, 2000);
	// chassis.turnToHeading(90, 2000);
	// chassis.moveToPoint(-66, 132, 2000, {.forwards=false});
	// chassis.waitUntilDone();
	// subsystem.pneumatics.toggleLatch();

	// chassis.moveToPoint(0, 120, 3000);
	// chassis.moveToPoint(24, 126, 3000);
	// chassis.moveToPoint(66, 132, 3000);

	// chassis.moveToPoint(24, 96, 3000);

}

void Autonomous::Test() {
	chassis.setPose(0, 0, 0);
	// chassis.moveToPoint(0, 24, 3000);
	chassis.turnToHeading(180, 3000);
	// chassis.moveToPoint(0, 0, 3000);
}

void Autonomous::AutoDrive()
{
	switch (active_route) {
		case (RED_NEG):
			RedNegative(); 
			break;
		case (RED_POS):
			RedPositive();
			break;
		case (BLUE_NEG):
			BlueNegative();
			break;
		case (BLUE_POS):
			BluePositive();
			break;
		case (TEST):
			Test();
			break;
		case (SKILLS):
			Skills();
			break;
		default:
			break;
	}
}