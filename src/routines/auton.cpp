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

        if ((fabs(error) < 3) or (pros::millis() - start_time > timeout)) { 
            left_motors.brake();
			right_motors.brake();
            break;
        }
        
        pros::delay(20);
    }
	}
}

void Autonomous::ClampPID(float target_distance, int timeout, bool followthrough=true) {
	int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (distance_sensor.get());

        double output = distance_sensor_pid.update((double) error);

        left_motors.move(output);
		right_motors.move(output);

        if ((fabs(error) < 15) or (pros::millis() - start_time > timeout)) { 
			if (followthrough) {
				left_motors.move(-80);
				right_motors.move(-80);
				pros::delay(100);
				subsystem.pneumatics.toggleLatch();
				pros::delay(100);
			}
            left_motors.brake();
			right_motors.brake();
			if (!followthrough) 
				subsystem.pneumatics.toggleLatch();
            break;
        }
        
        pros::delay(20);
    }
}

void Autonomous::RedNegative() {
	chassis.setPose(-24, -3, 180);

	subsystem.autonomous.ClampPID(40, 1500);
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

void Autonomous::RedPositive() {
	chassis.setPose(12.5, -12, -135);
	subsystem.arm.PID(15400, 500);
	chassis.moveToPoint(24, 0, 1500, {.maxSpeed=50});
	chassis.waitUntilDone();
	pros::delay(300);
	subsystem.arm.PID(1400, 500);
	chassis.turnToPoint(24, 24, 1500, {.forwards=false, .maxSpeed=50});

	//backwards autoclamp
	ClampPID(40, 2500);
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
	chassis.moveToPoint(6, -2, 1500, {.maxSpeed=50});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(12, 0, 1000, {.forwards=false, .maxSpeed=70});
	chassis.turnToPoint(0, 24, 750, {.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, 24, 3000, {.maxSpeed=30});
}
void Autonomous::BlueNegative() {
	chassis.setPose(24, -3, -180);

	subsystem.autonomous.ClampPID(40, 1500);
	pros::delay(250);

	chassis.moveToPoint(24, 32, 1500);
	chassis.turnToHeading(40, 1000, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPose(66, 37.8, 90, 2750, {.lead=0.8, .maxSpeed=80, .minSpeed=55, });
	chassis.waitUntilDone();
	chassis.moveToPose(24, 24, 55, 2000, {.forwards = false, .maxSpeed=100, .minSpeed=45});
	chassis.turnToPoint(46, 20, 1000);
	chassis.moveToPoint(46, 20, 1500, {.maxSpeed = 100});
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToPoint(0, 0, 750);
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(0, -4, 1500, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(8, 2, 1000, {.forwards=false});
}

void Autonomous::BluePositive() {
	chassis.setPose(-12.5, -12, 135);
	subsystem.arm.PID(15400, 500);
	chassis.moveToPoint(-24, 0, 1500, {.maxSpeed=50});
	chassis.waitUntilDone();
	pros::delay(300);
	subsystem.arm.PID(1400, 500);
	chassis.turnToPoint(-24, 24, 1500, {.forwards=false, .maxSpeed=50});

	//backwards autoclamp
	ClampPID(40, 2500);
	// subsystem.pneumatics.toggleLatch();

	chassis.turnToPoint(-48, 24, 750, {.maxSpeed=70});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	chassis.moveToPoint(-44, 24, 1500, {.maxSpeed=70});
	chassis.waitUntilDone();
	pros::delay(750);

	chassis.turnToPoint(-34, 0, 750, {.maxSpeed=60});
	chassis.moveToPoint(-34, 0, 1500, {.maxSpeed=60});
	chassis.turnToPoint(0, 0, 750, {.maxSpeed=60});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(-6, -3, 1500, {.maxSpeed=50});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleIntakeLift();
	chassis.moveToPoint(-12, 0, 1000, {.forwards=false, .maxSpeed=70});
	chassis.turnToPoint(0, 24, 750, {.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, 24, 3000, {.maxSpeed=30});
}

// Skills
void Autonomous::Skills() 
{
	chassis.setPose(0, -11.5, 0);
	subsystem.intake.toggle();
	pros::delay(750);
	chassis.moveToPoint(0, 0, 1000, {.maxSpeed=40});
	chassis.turnToPoint(-24, 0, 1000, {.forwards=false, .maxSpeed=70});
	ClampPID(13, 2500, false);
	// subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(500);
	subsystem.intake.toggle();
	chassis.turnToPoint(-48, 72, 1000);
	chassis.moveToPoint(-48, 72, 2000, {.maxSpeed=70});
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToPoint(-56, 48, 1000);
	chassis.moveToPoint(-56, 48, 1500, {.maxSpeed=40});
	pros::delay(500);
	subsystem.arm.PID(4250, 750);
	chassis.moveToPose(-52, 52, -90, 1250, {.forwards=false, .lead=0.75, .maxSpeed=60});
	chassis.waitUntilDone();
	chassis.turnToPoint(-66, 48, 750);
	subsystem.intake.toggle();
	chassis.moveToPoint(-66, 48, 2000, {.maxSpeed=30});
	pros::delay(750);
	subsystem.arm.PID(15400, 1500);
	pros::delay(200);
	chassis.moveToPoint(-47, 48, 1000, {.forwards=false, .maxSpeed=60});
	pros::delay(500);
	chassis.waitUntilDone();
	subsystem.arm.PID(1400, 500);
	subsystem.intake.toggle();
	chassis.turnToPoint(-50, -12, 1000, {.maxSpeed=80});
	chassis.moveToPoint(-50, -12, 2000, {.maxSpeed=50});
	// chassis.waitUntilDone();
	// chassis.setPose(-48, -7.5, 180);
	chassis.turnToPoint(-60, 2, 1000, {.maxSpeed=80});
	chassis.moveToPoint(-60, 2, 1500, {.maxSpeed=60});
	chassis.turnToPoint(-64, -12, 1000, {.forwards=false, .maxSpeed=80});
	chassis.moveToPoint(-64, -12, 750, {.forwards=false, .maxSpeed=50});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch();
	pros::delay(500);
	subsystem.intake.toggle();

	chassis.moveToPoint(0, 0, 2000, {.maxSpeed=70});
	chassis.waitUntilDone();
	pros::delay(100);
	chassis.turnToHeading(-90, 1000, {.maxSpeed=50});
	chassis.waitUntilDone();
	ClampPID(13, 1500, false);
	// subsystem.pneumatics.toggleLatch();

	subsystem.intake.toggle();
	pros::delay(500);
	subsystem.intake.toggle();
	chassis.turnToPoint(48, 72, 1000);
	chassis.moveToPoint(48, 72, 2000, {.maxSpeed=70});
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToPoint(56, 48, 1000);
	chassis.moveToPoint(56, 48, 1500, {.maxSpeed=40});
	pros::delay(500);
	subsystem.arm.PID(4250, 750);
	chassis.moveToPose(52, 52, 90, 1250, {.forwards=false, .lead=0.75, .maxSpeed=60 });
	chassis.waitUntilDone();
	chassis.turnToPoint(66, 48, 750);
	subsystem.intake.toggle();
	chassis.moveToPoint(66, 48, 2000, {.maxSpeed=30});
	pros::delay(750);
	subsystem.arm.PID(15400, 1500);
	pros::delay(200);
	chassis.moveToPoint(47, 48, 1000, {.forwards=false, .maxSpeed=60});
	pros::delay(500);
	chassis.waitUntilDone();
	subsystem.arm.PID(1400, 500);
	subsystem.intake.toggle();
	chassis.turnToPoint(48, 0, 1000, {.maxSpeed=80});
	chassis.moveToPoint(48, -14, 2000, {.maxSpeed=50});
	// chassis.waitUntilDone();
	// chassis.setPose(48, -7.5, 180);
	chassis.turnToPoint(60, 2, 1000, {.maxSpeed=80});
	chassis.moveToPoint(60, 2, 1500, {.maxSpeed=60});
	chassis.turnToPoint(64, -12, 1000, {.forwards=false, .maxSpeed=80});
	chassis.moveToPoint(64, -12, 750, {.forwards=false, .maxSpeed=50});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch();
	pros::delay(500);
	// subsystem.intake.toggle();	

	chassis.moveToPoint(55, 48, 1000, {.earlyExitRange=36});
	chassis.moveToPoint(48, 72, 1500, {.earlyExitRange=36});
	chassis.moveToPoint(-24, 112, 2000, {.minSpeed=50, .earlyExitRange=36});
	chassis.moveToPoint(-56, 114, 2500, {.minSpeed=50});
	chassis.moveToPoint(0, 100, 2000, {.forwards=false});
	chassis.moveToPoint(56, 114, 3000, {.forwards = false});

	chassis.waitUntilDone();
	subsystem.arm.PID(15400, 500);
	chassis.moveToPoint(30, 78, 1000, {.minSpeed=50});
	chassis.turnToHeading(45, 1000);
	chassis.moveToPoint(0, 48, 5000, {.forwards=false, .maxSpeed=60});
	// 24,72
	// turn to 45
	// back up slow

}

void Autonomous::Test() {
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(90, 5000, {.maxSpeed=50});
	// chassis.moveToPoint(0, 24, 3000);
	// chassis.moveToPoint(24, 24, 3000, {.maxSpeed=60});
	// chassis.turnToHeading(0, 3000);
	// chassis.turnToPoint(0, 0, 2000);
	// chassis.moveToPoint(0, 0, 3000);
	// chassis.turnToPoint(-24, 24, 3000);
	// chassis.moveToPoint(-24, 24, 3000);
	// chassis.turnToHeading(90, 3000);
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