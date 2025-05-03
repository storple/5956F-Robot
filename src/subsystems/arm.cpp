#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm() : target_angle(0), pid_active(false) { ; }


void Arm::PID(float target_angle, int timeout) {

    int start_time = pros::millis();

    while (true) {
        long double current_angle = ArmMotor.get_position();

        long double error = target_angle - current_angle;

        // Compute the PID output
        double output = current_arm_pid.update((double) error);

        // Apply the output to the motor
        ArmMotor.move(output);
        ArmMotor2.move(output);

        // Break the loop if close enough to the target (within a small tolerance)
        if (fabs(error) < 5) {  // Adjust tolerance as needed
            ArmMotor.move(0);
            ArmMotor2.move(0);
            break;
        }
        
        if (pros::millis() - start_time > timeout) {
            ArmMotor.move(0);
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

        long double current_angle = ArmMotor.get_position();
        long double error = target_angle - current_angle;

        double output = current_arm_pid.update((double)error);

        ArmMotor.move(output);
        ArmMotor2.move(output);

        if (fabs(error) < 1 || pros::millis() - start_time > 1500) {
            ArmMotor.move(0);
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
        target_angle = 0;
    } 
    else if (mode == 0) {
        target_angle = 120;
    }
    else if (mode == 1) {
        target_angle = 400;
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
            target_angle = -1;
            ArmMotor.move(127);
            ArmMotor2.move(127);
        }
        else {
            if (target_angle == -1) {
                ArmMotor.move(-127);
                ArmMotor2.move(-127);
                
                int start_time = pros::millis();
                while (ArmMotor.get_position() > 5 && pros::millis() - start_time < 1500) {
                    pros::delay(20);
                }
                ArmMotor.move(0);
                ArmMotor2.move(0);
                pros::delay(100);
                ArmMotor.tare_position();
                ArmMotor2.tare_position();
            }
            setArmState(0);
        }
    } else {
        // manual control after PID has completed
        if (!pid_active) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                ArmMotor.move(127);
                ArmMotor2.move(127);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                ArmMotor.move(-127);
                ArmMotor2.move(-127);

                if (ArmMotor.get_position() < 10) {
                    ArmMotor.tare_position();
                    ArmMotor2.tare_position();
                }
            } else {
                ArmMotor.brake();
                ArmMotor2.brake();
            }
        }
    }
}
