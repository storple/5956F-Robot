#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm() : target_angle(1400), pid_active(false) { ; }

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

void Arm::update() {
    if (pid_active) {
        long double current_angle = arm_sensor.get_angle();
        long double error = target_angle - current_angle;

        // Compute the PID output
        double output = arm_pid.update((double)error);

        // Apply the output to the motor
        ArmMotor1.move(output);
        ArmMotor2.move(output);

        // Stop PID if within tolerance
        if (fabs(error) < 750) {
            ArmMotor1.move(0);
            ArmMotor2.move(0);
            pid_active = false; // Disable PID control
        }
    }

    pros::delay(20);
}

void Arm::setTarget(float target) {
    target_angle = target;
    pid_active = true;
}

void Arm::run() {
    // Handle input for toggling autoMode
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        autoMode = !autoMode;

        if (!autoMode) {
            // Enter manual mode, but first move to position 1400
            setTarget(1400);
            pid_active = true; // Enable PID to reach the position
        } else {
            currentState = 0; // Start at default autoMode state
        }
    }

    if (autoMode) {
        // Cycle through states in autoMode
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            currentState = (currentState + 1) % 2; // Toggle between 0 and 1
        }

        // Set targets based on the state
        switch (currentState) {
            case 0:
                setTarget(4050);
                break;
            case 1:
                setTarget(15400);
                break;
        }
    } else {
        // Manual control after PID has completed
        if (!pid_active) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                ArmMotor1.move(40);
                ArmMotor2.move(40);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                ArmMotor1.move(-40);
                ArmMotor2.move(-40);
            } else {
                ArmMotor1.brake();
                ArmMotor2.brake();
            }
        }
    }
}
