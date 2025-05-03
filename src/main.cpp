#include "main.h"
#include "pros/apix.h"
#include "localization_utils.h"

using namespace Robot;
using namespace Robot::Globals;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 */

Length getRotationDistance(const pros::Rotation * rotation) {
    const Length odom_circumference = 1.995_in * 2 * M_PI;
    float rotations = from_stDeg(static_cast<float>(rotation->get_position()) / 100.0) / rot;
    Length totalPosition = rotations * odom_circumference; 
    return totalPosition;
}

void calibrate_odometry_distance(){
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
        if(Robot::Globals::horizontalEnc.is_installed()){
            pros::lcd::print(0, "hor: %f", getRotationDistance(&Robot::Globals::horizontalEnc).convert(in)); 
        }
        if(Robot::Globals::verticalEnc.is_installed()){
            pros::lcd::print(0, "ver: %f", getRotationDistance(&Robot::Globals::verticalEnc).convert(in));
        }
        pros::delay(50);
        }
    });
}


void particle_filter_init(){
    
    if(inertial_sensor.is_installed()){
        // imu is present, try to calibrate
        chassis.calibrateIMU();
    }else{
        // imu is not connected, should warn the user that this is the case
        printf("IMU IS NOT CONNECTED!!!!");
    }

    Robot::Globals::horizontalEnc.set_data_rate(5);
    Robot::Globals::verticalEnc.set_data_rate(5);

    Robot::Globals::horizontalEnc.set_position(0);
    Robot::Globals::verticalEnc.set_position(0);


    particle_filter.addSensor(&front_distance_model);
    particle_filter.addSensor(&left_distance_model);
    particle_filter.addSensor(&right_distance_model);
    particle_filter.addSensor(&back_distance_model);

    particle_filter.initUniform(-70_in, -70_in, 70_in, 70_in);
    inertial_sensor.set_heading(90);

    pros::Task localization_task = pros::Task([&] {
      uint32_t start_time = 0;
      while (true) {
        start_time = pros::millis();

        particle_filter.update();
        const localization::Pose prediction = particle_filter.getPrediction();
        chassis.setPose(
            to_in(prediction.x),
            to_in(prediction.y),
            inertial_sensor.get_heading()
        );

        pros::c::task_delay_until(&start_time, 10);
      }
    });

    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}


void initialize() {
    chassis.calibrate();
    ArmMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ArmMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ArmMotor.tare_position();

    // particle_filter_init();

    // runScreen();

    // pros::Task screenTask([&] {
    //     while (true) {
    //         update_position_labels(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta); 
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         pros::delay(50);         
    //     }s
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

    while (true) {
        pros::lcd::print(5, "distance: %d", front_distance.get()); // x
        pros::delay(50);
    }

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

