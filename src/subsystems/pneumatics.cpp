#include "robot/pneumatics.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Pneumatics::Pneumatics() { ; }

void Pneumatics::run() {

  if (useAutoClamp && detectMogo() && !mogoGoalState) {
    Pneumatics::toggleLatch();
    mogoGoalState = true;
  }

  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { mogoGoalState = !mogoGoalState; }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Pneumatics::toggleLatch(); mogoGoalState = !mogoGoalState; pros::delay(500); }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Pneumatics::toggleDoinker2(); }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Pneumatics::toggleDoinker(); }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { Pneumatics::toggleIntakeLift(); }

  pros::delay(20);
}

bool Pneumatics::detectMogo() {
  return (distance_sensor.get() < 20);
}

void Pneumatics::toggleLatch() { LatchControl.toggle(); }
void Pneumatics::toggleDoinker() { Doinker.toggle(); }
void Pneumatics::toggleDoinker2() { Doinker2.toggle(); }
void Pneumatics::toggleIntakeLift() { IntakeLift.toggle(); }

