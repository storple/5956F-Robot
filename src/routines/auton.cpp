#include "robot/auton.h"
#include "main.h" // IWYU pragma: export

using namespace Robot;
using namespace Robot::Globals;

ASSET(abc_txt);

void Autonomous::moveMotors(int power, int timeout) {
	left_motors.move(power);
	right_motors.move(power);
	pros::delay(timeout);
	left_motors.brake();
	right_motors.brake();
}

void Autonomous::WallSensorPID(float target_distance, int timeout, bool back) {

	if (!back) {
    int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (wall_sensor.get());

        double output = wall_sensor_pid.update((double) error);

        left_motors.move(-output);
		right_motors.move(-output);

        if ((fabs(error) < 5) or (pros::millis() - start_time > timeout)) { 
            left_motors.brake();
			right_motors.brake();
            break;
        }
        
        pros::delay(20);
    }
	}

	else {
		
    int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (wall_sensor_back.get());

        double output = wall_sensor_pid.update((double) error);

        left_motors.move(output);
		right_motors.move(output);

        if ((fabs(error) < 3) or (pros::millis() - start_time > timeout)) { 
            left_motors.brake();
			right_motors.brake();
            break;
        }
        
        pros::delay(20);
    }
	}
}

void Autonomous::ClampPID(float target_distance, int timeout, bool followthrough=true) {
	int start_time = pros::millis();

    while (true) {

        long double error = target_distance - (distance_sensor.get());

        double output = distance_sensor_pid.update((double) error);

        left_motors.move(output);
		right_motors.move(output);

        if ((fabs(error) < 15) or (pros::millis() - start_time > timeout)) { 
			if (followthrough) {
				left_motors.move(-60);
				right_motors.move(-60);
				pros::delay(100);
				subsystem.pneumatics.toggleLatch();
				pros::delay(100);
			}
			if (!followthrough) {
				left_motors.move(-30);
				right_motors.move(-30);
				pros::delay(50);
				subsystem.pneumatics.toggleLatch();
				pros::delay(50);
			}
			left_motors.brake();
			right_motors.brake();
            break;
        }
        pros::delay(20);
    }
}

void Autonomous::LimitSwitchIntake(int timeout) {
	int start_time = pros::millis();

	while (true) {
		if (LimitSwitch.get_new_press() or pros::millis() - start_time > timeout) {
			break;
		}
		pros::delay(15);
	}
	subsystem.intake.toggle();
}


float Autonomous::distanceResetX() {
	constexpr float SENSOR_OFFSET_X = 6.125; // offset in inches

    float measured_distance = wall_sensor_back.get() / 25.4; // convert from mm to inches
    float adjusted_distance = 72 - (measured_distance + SENSOR_OFFSET_X);

    return adjusted_distance;
}

void Autonomous::AutoDrive()
{
	switch (active_route) {
		case (RED_NEG):
			RedNegative(); 
			break;
		case (RED_POS):
			RedPositive();
			break;
		case (BLUE_NEG):
			BlueNegative();
			break;
		case (BLUE_POS):
			BluePositive();
			break;
		case (RED_GOALRUSH):
			RedGoalRush();
			break;
		case (BLUE_GOALRUSH):
			BlueGoalRush();
			break;
		case (TEST):
			Test();
			break;
		case (SKILLS):
			Skills();
			break;
		default:
			break;
	}
}
