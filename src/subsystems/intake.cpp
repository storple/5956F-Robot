#include "robot/intake.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Intake::Intake() { }

bool Intake::detectRing() {
	return (color_sensor.get_proximity() >= 230);
}

bool Intake::detectBadColor() {
	// 250 +- 50 maybe for BLUE
	// 0-50 ish for RED
	double hue = color_sensor.get_hue();
	double proximity = color_sensor.get_proximity();

	return ((detectRing()) and ((0 <= hue and hue <= 50 and !playingRedSide) or (200 <= hue and hue <= 300 and playingRedSide)));
	
}

// if detect color is true, wait till limit switch is pressed, then cut velocity on intake for short amount of time (100 msec), return to business as normal

void Intake::run()
{

	// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
	// 	useColorSort = !useColorSort;
	// }

	if (useColorSort and detectBadColor()) {
		
		int start_time = pros::millis();

		IntakeMotor.move(-127);
		while (LimitSwitch.get_value() == 0) { 
			pros::delay(10);
			if (pros::millis() - start_time > 2000) break; // failsafe
        }
		pros::delay(100);
		IntakeMotor.brake();
		pros::delay(200);
	}
	
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		IntakeMotor.move(-127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		IntakeMotor.move(127);
	}
	else {
		IntakeMotor.brake();
	}

	pros::delay(20);
}

void Intake::run(int msec) {

	int start = pros::millis();

	while (pros::millis() - start < msec) {
		IntakeMotor.move(-127);
		pros::delay(20);
	}

	IntakeMotor.brake();
}

void Intake::toggle() {
	if (!currentlyOn) {
		IntakeMotor.move(-127);
	}
	else {
		IntakeMotor.brake();
	}
	currentlyOn = !currentlyOn;
}