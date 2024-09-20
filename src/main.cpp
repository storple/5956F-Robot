#include "main.h"
#include "pros/apix.h"

using namespace Robot;
using namespace Robot::Globals;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {

    std::cout << "initializing...\n";

    // arm_sensor.reset();
    // while (arm_sensor.is_calibrating()) {
    //     pros::delay(100);  // Wait for calibration
    // }

    ArmMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    ArmMotor.set_zero_position(0);
    ArmMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

    chassis.calibrate();

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    std::cout << "Done initializing.\n";
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

    subsystem.autonomous.AutoDrive(subsystem.intake, subsystem.latch);

    // set position to x:0, y:0, heading:0

    /*
    chassis.setPose(0, 0, 0);
    // move forwards
    chassis.moveToPoint(0, 100, 20000);
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(0, 50, 20000);
    */
}


/**
 * Runs the operator control code. 
 */
void opcontrol() {
    while (true) {

        controller.print(0, 0, std::to_string( chassis.getPose().x ).c_str());
        pros::delay(50);
        controller.print(1, 0, std::to_string( chassis.getPose().y ).c_str() );

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            autonomous();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 2000);
        }

        /* toggleable drive modes 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            std::string name = subsystem.drivetrain.toggleDrive();
        // Output the current drive mode to the controller screen
            controller.print(0, 0, name.c_str());
        }
        */

        subsystem.drivetrain.run();
        subsystem.latch.run();
        subsystem.intake.run();
        subsystem.arm.run();
        
        pros::delay(25);
    }
}