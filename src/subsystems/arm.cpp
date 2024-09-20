#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm()
{
    int currentState = 0;
}

void Arm::PID(float target_angle) {

    int start_time = pros::millis();

    while (true) {
        double current_angle = arm_sensor.get_pitch();

        current_angle = ArmMotor.get_position(); 

        double error = target_angle - current_angle;

        // Compute the PID output
        double output = arm_pid.update(error);

        // Apply the output to the motor
        ArmMotor.move(output);

        // Break the loop if close enough to the target (within a small tolerance)
        if (fabs(error) < 50.0) {  // Adjust tolerance as needed
            break;
        }
        
        if (pros::millis() - start_time > 2000) {
            break;
        }

        pros::delay(20);  // Small delay for loop stability
    }
}

void Arm::run()
{

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        ArmClaw.toggle();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        currentState = -1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        currentState = (currentState + 1) % 3;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        currentState = (((currentState-1) % 3) + 3) % 3;
    }

    switch (currentState) {
        case 0:
            PID(-175);
            break;
        case 1:
            PID(-1500);
            break;
        case 2:
            PID(-2800);
            break;
        default:
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
                ArmMotor.move(127);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) 
                ArmMotor.move(-127);
            else
                ArmMotor.brake();
            break;
    }

    pros::delay(15);
}

