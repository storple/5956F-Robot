#include "robot/pneumatics.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Pneumatics::Pneumatics() { ; }

void Pneumatics::run() {

  if (detectMogo() && !state) {
    Pneumatics::toggleLatch();
    state = true;
  }


  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Pneumatics::toggleLatch(); state = !state; pros::delay(500); }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Pneumatics::toggleDoinker(); }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { Pneumatics::toggleIntakeLift(); }

  pros::delay(20);
}

bool Pneumatics::detectMogo() {
  return (distance_sensor.get() < 25);
}

void Pneumatics::toggleLatch() { LatchControl.toggle(); }
void Pneumatics::toggleDoinker() { Doinker.toggle(); }
void Pneumatics::toggleIntakeLift() { IntakeLift.toggle(); }

