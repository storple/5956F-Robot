#include "liblvgl/core/lv_obj_class.h"
#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::Test() {
	localization::setPose(0_in, 0_in, 0, 2_in);
	chassis.turnToHeading(90, 5000, {}, false);
	// chassis.moveToPoint(0, 24, 3000);
	// chassis.moveToPoint(0, 48, 5000);
	// chassis.turnToHeading(0, 3000);
	// chassis.turnToPoint(0, 0, 2000);
	// chassis.moveToPoint(0, 0, 3000);
	// chassis.turnToPoint(-24, 24, 3000);
	// chassis.moveToPoint(-24, 24, 3000);
	// chassis.turnToHeading(90, 3000);
	// chassis.moveToPoint(0, 0, 3000);
}
