#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm() : target_angle(1400), pid_active(false) { ; }


void Arm::PID(float target_angle, int timeout) {

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
        
        if (pros::millis() - start_time > timeout) {
            ArmMotor1.move(0);
            ArmMotor2.move(0);
            break;
        }

        pros::delay(20);  // Small delay for loop stability
    }
}

void Arm::update() {
    static int start_time = 0;
    if (pid_active) {
        if (start_time == 0) { 
            start_time = pros::millis();
        }

        long double current_angle = arm_sensor.get_angle();
        long double error = target_angle - current_angle;

        double output = arm_pid.update((double)error);

        ArmMotor1.move(output);
        ArmMotor2.move(output);

        if (fabs(error) < 200 || pros::millis() - start_time > 800) {
            ArmMotor1.move(0);
            ArmMotor2.move(0);
            pid_active = false; // Disable PID control
            start_time = 0;     // Reset start_time for the next activation
        }
    }

    pros::delay(20);
}

/*
    arm states
    -1: down
    0: prep
    1: score/up
*/
void Arm::setArmState(int mode) {
    if (mode == -1) {
        target_angle = 1600;
    } 
    else if (mode == 0) {
        target_angle = 3400;
    }
    else if (mode == 1) {
        target_angle = 15400;
    }

    pid_active = true;

}

void Arm::run() {

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        autoMode = !autoMode;

        if (!autoMode) {
            setArmState(-1);
            pid_active = true; 
        } else {
            currentState = 0; 
        }
    }

    if (autoMode) {
        // cycle through states in autoMode
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            pid_active = false;
            ArmMotor1.move(127);
            ArmMotor2.move(127);
        }
        else {
            setArmState(0);
        }
    } else {
        // manual control after PID has completed
        if (!pid_active) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                ArmMotor1.move(127);
                ArmMotor2.move(127);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                ArmMotor1.move(-127);
                ArmMotor2.move(-127);
            } else {
                ArmMotor1.brake();
                ArmMotor2.brake();
            }
        }
    }
}
