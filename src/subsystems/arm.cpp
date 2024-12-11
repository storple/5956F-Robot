#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm() { ; }

void Arm::PID(float target_angle) {

    int start_time = pros::millis();

    while (true) {
        long double current_angle = arm_sensor.get_angle();

        long double error = target_angle - current_angle;

        // Compute the PID output
        double output = arm_pid.update((double) error);

        // Apply the output to the motor
        ArmMotor1.move(output);
        ArmMotor2.move(output);

        // Break the loop if close enough to the target (within a small tolerance)
        if (fabs(error) < 750) {  // Adjust tolerance as needed
            ArmMotor1.move(0);
            ArmMotor2.move(0);
            break;
        }
        
        if (pros::millis() - start_time > 500) {
            ArmMotor1.move(0);
            ArmMotor2.move(0);
            break;
        }

        pros::delay(20);  // Small delay for loop stability
    }
}

void Arm::run()
{

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        currentState = (currentState == -1) ? 0 : -1;
        autoMode = !autoMode;
        pressed = true; 
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) and autoMode) {
        currentState = (currentState + 1) % 2;
    }
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    //     currentState = (((currentState-1) % 2) + 3) % 2;
    // }

    // pros::delay(50);
    // pros::lcd::print(6, "currentState: %d", currentState);
    switch (currentState) {
        case 0:
            PID(3800);
            break;
        case 1:
            PID(15400);
            break;
        default:
            if (pressed) {
                // pros::delay(50);
                // pros::lcd::print(5, "Executed %f", pros::millis());
                PID(1400);
                pressed = false;
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                ArmMotor1.move(40);
                ArmMotor2.move(40);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                ArmMotor1.move(-40);
                ArmMotor2.move(-40);
            }
            else {
                ArmMotor1.brake();
                ArmMotor2.brake();
            }
            break;
    }

    pros::delay(20);
}

