#include "robot/auton.h"
#include "main.h" // IWYU pragma: export

using namespace Robot;
using namespace Robot::Globals;

ASSET(abc_txt);

void Autonomous::Auton1()
{
	
	chassis.setPose(0, 0, 0);
	// chassis.moveToPoint(0, -24, 1500, {.forwards=false});
	// pros::delay(650);

	left_motors.move(-60);
	right_motors.move(-60);

	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(50);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();

	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(500);
	chassis.turnToHeading(170, 3000);
	chassis.moveToPose(35, -52, 90, 3000, {.maxSpeed=60});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, -24, 3000, {.forwards=false});
	chassis.waitUntilDone();
	chassis.moveToPoint(30, -24, 3000);
	chassis.waitUntilDone();
	chassis.moveToPoint(48, 24, 2000);
	chassis.waitUntilDone();

	start_time = pros::millis();

	while (!subsystem.intake.detectRing()) { 
		pros::delay(20);
		if (pros::millis() - start_time > 2200) break; // failsafe
	}

	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch();
	chassis.moveToPoint(-18, 12, 3000);
	chassis.turnToHeading(180, 3000);
	
	chassis.moveToPoint(-14, 11, 3000, {.forwards=false});
	pros::delay(500);
	subsystem.intake.toggle();
	pros::delay(3000);
	subsystem.intake.toggle();
	// subsystem.intake.toggle()
	// chassis.moveToPoint(-18, -30, 3000);


	
}

void Autonomous::Auton2()
{
	chassis.setPose(0, 0, 0);
	
	chassis.moveToPoint(0, -12, 3000, {.forwards=false});
	chassis.waitUntilDone();
	
	chassis.turnToHeading(-90, 3000);
	chassis.moveToPoint(5, -12, 3000, {.forwards=false});
	subsystem.intake.toggle();
	pros::delay(1500);
	chassis.moveToPoint(-5, -12, 1000);
	chassis.moveToPoint(-7, -12, 1000);
	
	pros::delay(1000);
	subsystem.intake.toggle();
	chassis.turnToHeading(-130, 1000);
	chassis.waitUntilDone();
	

	left_motors.move(-60);
	right_motors.move(-60);

	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(50);
		if (pros::millis() - start_time > 1500) break;
	}

	subsystem.pneumatics.toggleLatch();

	left_motors.brake();
	right_motors.brake();

	chassis.turnToHeading(0, 2000);
	chassis.moveToPoint(-15, 30, 3000);

	chassis.turnToHeading(-130, 3000);

	chassis.moveToPoint(15, 45, 3000, {.forwards=false});
	chassis.waitUntilDone();

	subsystem.pneumatics.toggleLatch();

	chassis.moveToPoint(0, 40, 3000);
	chassis.turnToHeading(90, 3000);
	chassis.moveToPoint(-15, 40, 3000);
	chassis.waitUntilDone();

	subsystem.pneumatics.toggleLatch();


}

void Autonomous::Auton3()
{
	chassis.setPose(0, 0, 0);


	// chassis.moveToPoint(0, -24, 1500, {.forwards=false});
	// pros::delay(650);

	left_motors.move(-60);
	right_motors.move(-60);

	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(50);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();

	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();
	pros::delay(500);
	chassis.turnToHeading(-170, 3000);
	chassis.moveToPose(-35, -52, 90, 3000, {.maxSpeed=60});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, -24, 3000, {.forwards=false});
	chassis.waitUntilDone();
	chassis.moveToPoint(-30, -24, 3000);
	chassis.waitUntilDone();
	chassis.moveToPoint(-48, 24, 2000);
	chassis.waitUntilDone();

	start_time = pros::millis();

	while (!subsystem.intake.detectRing()) { 
		pros::delay(20);
		if (pros::millis() - start_time > 2200) break; // failsafe
	}

	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch();
	chassis.moveToPoint(18, 12, 3000);
	chassis.turnToHeading(180, 3000);
	
	chassis.moveToPoint(14, 11, 3000, {.forwards=false});
	pros::delay(500);
	subsystem.intake.toggle();
	pros::delay(3000);
	subsystem.intake.toggle();
}

void Autonomous::Auton4()
{
  int turnVoltage = 50;
  float time = 5000;
  chassis.setPose(0, 0, 0);
  unsigned long n = 0;
  float heading;

  std::cout << std::fixed << "\033[1mCopy this:\033[0m\n\\left[";
  right_motors.move_voltage(8000);
  left_motors.move_voltage(-8000);

  auto end_time = time + pros::millis();

  int i = 0;
  
  while (pros::millis() < end_time && i++ < 10000) {
    std::cout << "\\left(" << chassis.getPose().x << "," << chassis.getPose().y << "\\right),";
    /*if (i % 250 == 0) {
      std::cout << "\\right]\n\\left[" ;
    } */
    if (i % 50 == 0) {
      std::cout.flush();
    }
    pros::delay(20);
  }  
  chassis.cancelAllMotions();
  std::cout << "\b\\right]" << std::endl;

  std::cout << "Go to https://www.desmos.com/calculator/rxdoxxil1j to solve for offsets." << std::endl;
}

// Skills
void Autonomous::Skills() 
{
	chassis.setPose(0, 0, 0);
	subsystem.intake.toggle();
	pros::delay(500);
	chassis.turnToHeading(105, 1000);
	chassis.waitUntilDone();
	
	// go backwards to mogo
	left_motors.move(-60);
	right_motors.move(-60);

	int start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(60);
		if (pros::millis() - start_time > 1500) break;
	}

	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();
	
	chassis.turnToHeading(10, 1000);
	chassis.moveToPose(-24, 28, -20,1000);
	chassis.moveToPoint(-48, 88, 2000);
	chassis.turnToHeading(184, 1000);
	chassis.waitUntilDone();
	subsystem.arm.PID(3800);

	chassis.moveToPoint(-60, 54, 1500);
	chassis.waitUntilDone();

	// start_time = pros::millis();
	// subsystem.arm.PID(3800);
	// left_motors.move(100);
	// right_motors.move(100);
	// while (wall_sensor.get() > 1650) {
	// 	pros::delay(20);
	// 	if (pros::millis() - start_time > 2000) break;
	// }
	// left_motors.brake();
	// right_motors.brake();

	chassis.turnToHeading(-90, 1000);
	chassis.waitUntilDone();

	// chassis.moveToPoint(-68, 64, 1000);
	// chassis.waitUntilDone();
	
	start_time = pros::millis();
	subsystem.arm.PID(3800);
	left_motors.move(30);
	right_motors.move(30);
	while (wall_sensor.get() > 148) {
		pros::delay(20);
		if (pros::millis() - start_time > 2000) break;
	}
	left_motors.brake();
	right_motors.brake();
	
	pros::delay(500);
	subsystem.intake.toggle();
	pros::delay(700);
	subsystem.arm.PID(15400);

	start_time = pros::millis();
	left_motors.move(-30);
	right_motors.move(-30);
	while (wall_sensor.get() < 400) {
		pros::delay(20);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();

	// chassis.moveToPoint(-44, 60, 1000, {.forwards=false});
	// chassis.waitUntilDone();

	subsystem.intake.toggle();
	subsystem.arm.PID(1400);
	chassis.turnToHeading(150, 1000);
	chassis.moveToPoint(-48, -10, 3000, {.maxSpeed=50});
	chassis.moveToPoint(-52, 10, 2500);
	chassis.waitUntilDone();
	chassis.moveToPoint(-68, -10, 2000, {.forwards=false});
	subsystem.intake.toggle();
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleLatch();
	

	chassis.moveToPoint(4, 10, 3000);
	chassis.turnToHeading(-90, 1000);
	chassis.waitUntilDone();
	subsystem.intake.toggle();

	left_motors.move(-60);
	right_motors.move(-60);

	start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(60);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();


	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint(16, 24, 1000);
	chassis.moveToPoint(40, 88, 2000);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(42, 57, 1000);
	subsystem.arm.PID(3800);
	chassis.turnToHeading(90, 1000);

	chassis.moveToPoint(56, 62, 1000);
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	pros::delay(500);
	subsystem.arm.PID(15400);

	subsystem.intake.toggle();

	chassis.moveToPoint(56, 66, 1000, {.forwards=false});
	subsystem.arm.PID(1400);
	chassis.turnToHeading(-150, 1000);
	chassis.moveToPoint(48, -10, 3000);
	chassis.moveToPoint(56, 0, 1000);
	chassis.moveToPoint(72, -10, 2000, {.forwards=false});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleLatch();

	chassis.turnToHeading(0, 3000);
	chassis.moveToPose(24, 96, -30, 3000);

	start_time = pros::millis();
	while (!subsystem.intake.detectRing()) { 
		pros::delay(20);
		if (pros::millis() - start_time > 2200) break; // failsafe
	}
	subsystem.intake.toggle();

	chassis.turnToHeading(135, 3000);
	
	left_motors.move(-60);
	right_motors.move(-60);

	start_time = pros::millis();
	while (!subsystem.pneumatics.detectMogo()) {
		pros::delay(60);
		if (pros::millis() - start_time > 1500) break;
	}
	left_motors.brake();
	right_motors.brake();
	subsystem.pneumatics.toggleLatch();
	subsystem.intake.toggle();

	chassis.turnToHeading(-135, 3000);
	chassis.moveToPoint(-24, 96, 3000);
	
	chassis.turnToHeading(-30, 3000);
	chassis.moveToPoint(-36, 120, 300);
	chassis.moveToPoint(-66, 120, 1000);
	chassis.moveToPoint(-48, 132, 2000);
	chassis.turnToHeading(90, 2000);
	chassis.moveToPoint(-66, 132, 2000, {.forwards=false});
	chassis.waitUntilDone();
	subsystem.pneumatics.toggleLatch();

	chassis.moveToPoint(0, 120, 3000);
	chassis.moveToPoint(24, 126, 3000);
	chassis.moveToPoint(66, 132, 3000);

	chassis.moveToPoint(24, 96, 3000);

}

void Autonomous::AutoDrive()
{
	switch (active_route) {
		case (RED_NEG):
			Auton1(); 
			break;
		case (RED_POS):
			Auton2();
			break;
		case (BLUE_NEG):
			Auton3();
			break;
		case (BLUE_POS):
			Auton4();
			break;
		case (SKILLS):
			Skills();
			break;
		default:
			Auton1();
			break;
	}
}