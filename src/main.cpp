#include "main.h"
#include "pros/apix.h"

using namespace Robot;
using namespace Robot::Globals;

/**
 * @brief Structure that holds instances of all robot subsystems.
 */
struct RobotSubsystems {
  Robot::Autonomous autonomous;
  Robot::Drivetrain drivetrain;
  Robot::Intake intake;
  Robot::Latch latch;
} subsystem;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
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
    // subsystem.autonomous.AutoDrive(subsystem.intake, subsystem.latch);
}

/**
 * Runs the operator control code. 
 */
void opcontrol() {
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            autonomous();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            std::string name = subsystem.drivetrain.toggleDrive();
        // Output the current drive mode to the controller screen
            controller.print(0, 0, name.c_str());
        }

        subsystem.drivetrain.run();
        subsystem.latch.run();
        subsystem.intake.run();
        
        pros::delay(25);
    }
}