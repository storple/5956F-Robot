#include "robot/intake.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Intake::Intake()
{
}

void Intake::run()
{
	
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		IntakeMotor.move(-127);
	}
	else {
		IntakeMotor.brake();
	}

}