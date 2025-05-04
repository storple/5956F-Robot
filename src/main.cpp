#include "main.h"
#include "pros/apix.h"

using namespace Robot;
using namespace Robot::Globals;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 */

void initialize() {

    chassis.calibrate();
    ArmMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ArmMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ArmMotor.tare_position();

    // runScreen();

    // pros::Task screenTask([&] {
    //     while (true) {
    //         update_position_labels(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta); 
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         pros::delay(50);         
    //     }
    // });

    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
        // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "armAngle: %f", ArmMotor.get_position());
            pros::lcd::print(5, "armVoltage: %d", ArmMotor.get_voltage());
            // pros::lcd::print(4, "autoClamp: %d", useAutoClamp);
            // // pros::lcd::print(5, "distance: %d", wall_sensor_back.get()); // x
            // // pros::lcd::print(6, "useColorSort: %d", useColorSort);
            // // pros::lcd::print(7, "ring distance: %d", color_sensor.get_proximity());

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });


}

/* Runs while the robot is disabled */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
    subsystem.autonomous.AutoDrive();
}


/**
 * Runs the operator control code. 
 */
void opcontrol() {
   
   // autonomous();

    // if (subsystem.pneumatics.detectMogo()) 
    //     mogoGoalState = true;
    // else
    //     mogoGoalState = false;

    pros::Task drivetrainTask([] { while (true) { subsystem.drivetrain.run(); } });
    pros::Task pneumaticsTask([] { while (true) { subsystem.pneumatics.run(); } });
    pros::Task intakeTask([] { while (true) { subsystem.intake.run(); } });
    pros::Task armTask([] { while (true) { subsystem.arm.run(); subsystem.arm.update(); } });

    // controller.print(0, 0, ("colorsort: " + std::to_string(useColorSort)).c_str());

    // while (true) {
    //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //         autonomous();
    //     }
    //     pros::delay(100);
    // }


    // while (true) {

    //     controller.print(0, 0, std::to_string( chassis.getPose().x ).c_str());
    //     pros::delay(50);
    //     controller.print(1, 0, std::to_string( chassis.getPose().y ).c_str() );

    //     // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    //     //     autonomous();
    //     // }
    //     // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //     //     chassis.setPose(0, 0, 0);
    //     //     chassis.turnToHeading(90, 2000);
    //     // }

    //     /* toggleable drive modes 
    //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //         std::string name = subsystem.drivetrain.toggleDrive();
    //     // Output the current drive mode to the controller screen
    //         controller.print(0, 0, name.c_str());
    //     }
    //     */
        
    //     pros::delay(25);
    // }
}

