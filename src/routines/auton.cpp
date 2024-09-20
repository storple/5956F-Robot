#include "robot/auton.h"
#include "main.h" // IWYU pragma: export

using namespace Robot;
using namespace Robot::Globals;

Autonomous::AUTON_ROUTINE Autonomous::auton = RED_POS;
std::string				  Autonomous::autonName;

ASSET(blue_path_txt);

// BLUE POSITIVE
void Autonomous::Auton1(Intake &intake, Latch &latch)
{
	// initialize
	chassis.setPose(0, 0, 0);
	subsystem.arm.PID(-500);


	// move towards wall stake
	chassis.moveToPoint(0.03, 32, 2000);
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPoint(12, 32, 2000, {.forwards=false});
	chassis.turnToHeading(-90, 500);

	// score on wall stake
	subsystem.intake.toggle(127);
	pros::delay(1000);
	 chassis.moveToPoint(25, 32, 1000, {.forwards=false});
	chassis.moveToPoint(0.3, 34, 1000);
	subsystem.intake.toggle(0);

	// pick up mobile goal
	chassis.turnToHeading(55, 1000);
	subsystem.latch.toggle();
	chassis.moveToPoint(-40.07, 1, 3000,{.forwards=false, .maxSpeed=50}, false);
	subsystem.latch.toggle();
	pros::delay(500);

	// score ring
	chassis.turnToHeading(180,1000);
	subsystem.intake.toggle(100);
    chassis.moveToPoint(-44.07, -32, 2000);

	// move towards ladder
	pros::delay(1000);
	chassis.turnToHeading(90, 1000);
	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	subsystem.arm.PID(-1000);
	chassis.moveToPoint(-42.07,13,2000);

}


// BLUE NEGATIVE
void Autonomous::Auton2(Intake &intake, Latch &latch) {

}

// RED POSITIVE
void Autonomous::Auton3(Intake &intake, Latch &latch) {
	// initialize
	chassis.setPose(0, 0, 0);
	subsystem.arm.PID(-500);

	// move to wall stake
	chassis.moveToPoint(0.03, 32, 2000);
	chassis.turnToHeading(90, 1000);
	chassis.moveToPoint(-12, 32, 2000, {.forwards=false});
	chassis.turnToHeading(90, 500);

	// score on wall stake
	subsystem.intake.toggle(127);
	pros::delay(1000);
	 chassis.moveToPoint(25, 32, 1000, {.forwards=false});
	chassis.moveToPoint(0.3, 34, 1000);
	subsystem.intake.toggle(0);

	// move towards mobile goal
	chassis.turnToHeading(55, 1000);
	subsystem.latch.toggle();
	chassis.moveToPoint(-40.07, 1, 3000,{.forwards=false, .maxSpeed=50}, false);
	subsystem.latch.toggle();
	pros::delay(500);

	// score ring
	chassis.turnToHeading(180,1000);
	subsystem.intake.toggle(100);

	// move towards ladder
    chassis.moveToPoint(-44.07, -32, 2000);
	pros::delay(1000);
	chassis.turnToHeading(90, 1000);
	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	subsystem.arm.PID(-1000);
	chassis.moveToPoint(-42.07,13,2000);
}

// RED NEGATIVE
void Autonomous::Auton4(Intake &intake, Latch &latch)
{
	// initialize
	chassis.setPose(0, 0, 0);
	subsystem.arm.PID(-500);

	// move to wall stake
	chassis.moveToPoint(0.03, 32, 2000);
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPoint(12, 32, 700, {.forwards=false});
	chassis.turnToHeading(-90, 500);

	// head towards wall stake
	subsystem.intake.toggle(127);
	pros::delay(700);
	 chassis.moveToPoint(25, 32, 1000, {.forwards=false});
	 chassis.moveToPoint(0.3, 34, 1000);
	subsystem.intake.toggle(0);

	// get mobile goal
	chassis.turnToHeading(55, 1000);
	subsystem.latch.toggle();
	chassis.moveToPoint(-40.07, 1, 3000,{.forwards=false, .maxSpeed=60}, false);
	subsystem.latch.toggle();
	pros::delay(500);

	
	// score rings
	chassis.turnToHeading(180,1000);
	subsystem.intake.toggle(100);
    chassis.moveToPoint(-44.07, -32, 2000);
	pros::delay(1000);
	chassis.turnToHeading(-100, 1000);
	chassis.moveToPoint(-60.07, -38, 2000);

	// move towards ladder
	chassis.turnToHeading(-10, 1000);
	subsystem.arm.PID(-2000);
	subsystem.arm.currentState = 2;
	chassis.moveToPoint(-60,-8.5,2000);

}

void Autonomous::AutoDrive(Intake &intake, Latch &latch)
{

	Auton4(intake, latch);

	// Keep the switcher running while the controller down button has not been pressed and the time period is not
	// autonomous Compare the current auton value to run the auton routine
	// switch (Autonomous::auton) {
	// case TEST:
	// 	Auton1(intake, latch);
	// 	break;
	// }
}

void Autonomous::AutonSwitcher(int autonNum)
{
	switch (autonNum) {
	case 1:
		Autonomous::autonName = "Blue Positive";
		Autonomous::auton	  = BLUE_POS;
		break;
	case 2:
		Autonomous::autonName = "Blue Negative";
		Autonomous::auton	  = BLUE_NEG;
		break;
	case -1:
		Autonomous::autonName = "Red Positive";
		Autonomous::auton	  = RED_POS;
		break;
	case -2:
		Autonomous::autonName = "Red Negative";
		Autonomous::auton	  = RED_NEG;
		break;
	case 0:
		Autonomous::autonName = "Skills";
		Autonomous::auton	  = SKILLS;
	}
	std::cout << "Current auton: " + Autonomous::autonName << std::endl;
}