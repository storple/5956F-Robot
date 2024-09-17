#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm()
{
    int currentState = 0;
}

void Arm::PID(float target_angle) {
    while (true) {
        double current_angle = arm_sensor.get_pitch();
        current_angle = ArmMotor.get_position(); 

        controller.print(0, 0, std::to_string(current_angle).c_str());
        double error = target_angle - current_angle;

        // Compute the PID output
        double output = arm_pid.update(error);

        // Apply the output to the motor
        ArmMotor.move(output);

        // Break the loop if close enough to the target (within a small tolerance)
        if (fabs(error) < 50.0) {  // Adjust tolerance as needed
            break;
        }
        std::cout << "looping " << error << "\n";
        pros::delay(20);  // Small delay for loop stability
    }
}

void Arm::run()
{

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        ArmClaw.toggle();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        currentState = (currentState+1) % 4;
        // controller.print(0, 0, std::to_string(currentState).c_str());
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) currentState = 4;

    switch (currentState) {
        case 0:
            PID(-150);
            break;
        case 1:
            PID(-1500);
            break;
        case 2:
            PID(-3000);
            break;
        case 3:
            PID(-4500);
            break;
        default:
            controller.print(0, 0, (std::to_string(ArmMotor.get_position()).c_str()));
            // controller.print(1, 0,  std::to_string(arm_sensor.get_yaw()).c_str());
            // PID(0);
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                ArmMotor.move(127);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                ArmMotor.move(-127);
            }
            else {
                ArmMotor.brake();
            }
            break;
    }

    pros::delay(15);
}

