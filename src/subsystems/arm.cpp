#include "robot/arm.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

Arm::Arm()
{
    int currentState = 0;
    float current = 0;    // Current position from sensor
    float error = 0;
    float lastError = 0;
    float integral = 0;
    float derivative = 0;

    // PID constants
    float kP = 0.5;
    float kI = 0.01;
    float kD = 0.1;
}

void Arm::PID(float target) {
    
    current = arm_sensor.get_pitch();

    error = target - current;
    integral += error;
    derivative = error - lastError; 

    // PID formula
    float output = (kP * error) + (kI * integral) + (kD * derivative);

    // Send the output to the motor
    ArmMotor.move(output);

    // Save the current error for the next loop
    lastError = error;
}

void Arm::run()
{

    // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    //     ArmMotor.move(127);
    //     controller.print(0, 0, "arm up");
    // }
    // else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    //     ArmMotor.move(-127);
    //     controller.print(0, 0, "arm down");
    // }
    // else {
    //     ArmMotor.brake();
    //     controller.print(0, 0, "arm idle");
    // }

    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    //     ArmClaw.toggle();
    // }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        currentState = (currentState+1) % 3;
        controller.print(0, 0, "switched");
    }

    switch (currentState) {
        case 0:
            PID(43);
            break;
        case 1:
            PID(-35);
            break;
        case 2:
            PID(60);
            break;
    }

    pros::delay(15);
}

