#include "robot/intake.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Intake::Intake()
{
	bool currentlyOn = false;
}

void Intake::run()
{
	
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		IntakeMotor.move(-127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		IntakeMotor.move(127);
	}
	else {
		IntakeMotor.brake();
	}

}

void Intake::run(int msec) {

	int start = pros::millis();

	while (pros::millis() - start < msec) {
		IntakeMotor.move(-127);
		pros::delay(20);
	}

	IntakeMotor.brake();
}

void Intake::toggle(int speed) {
	if (!currentlyOn) {
		IntakeMotor.move(-speed);
	}
	else {
		IntakeMotor.brake();
	}
	currentlyOn = !currentlyOn;
}