#include "robot/auton.h"
#include "main.h" // IWYU pragma: export

using namespace Robot;
using namespace Robot::Globals;

ASSET(abc_txt);

void Autonomous::Auton1()
{
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 48, 3000);
	// chassis.turnToHeading(90, 5000);
	// chassis.moveToPoint(0, x, 5000);
	// x += 5;
}

void Autonomous::Auton2()
{
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(90, 5000);

}

void Autonomous::Auton3()
{
	pros::lcd::print(3, "Auton Activated"); // heading
	chassis.setPose(-35.5, -47, 0);
	chassis.follow(abc_txt, 10, 20000);
	pros::lcd::print(3, "Auton Finished");
}

void Autonomous::Auton4()
{
	
}

// Skills
void Autonomous::Skills() 
{

}

void Autonomous::AutoDrive()
{
	switch (active_route) {
		case (RED_LEFT):
			Auton1(); 
			break;
		case (RED_RIGHT):
			Auton2();
			break;
		case (BLUE_LEFT):
			Auton3();
			break;
		case (BLUE_RIGHT):
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