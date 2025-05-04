#include "globals.h"
#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::Test() {
	chassis.setPose(0, 0, 0);


	// auto last_vertical_position = verticalEnc.get_position();
	// auto last_horizontal_position = horizontalEnc.get_position();
	// auto last_imu = inertial_sensor.get_rotation();
	// verticalEnc.set_data_rate(5);
	// horizontalEnc.set_data_rate(5);
	// printf("\\left[");
	// for(int i = 0;i < 150;i++){
	// 	left_motors.move(100);
	// 	right_motors.move(-100);
	// 	auto curr_vertical_position =   verticalEnc.get_position();
	// 	auto curr_horizontal_position = horizontalEnc.get_position();
	// 	auto curr_imu = inertial_sensor.get_rotation();
		
	// 	auto imu_change = (M_PI / 180.0) * static_cast<float>(curr_imu - last_imu);
	// 	auto ver_change = (static_cast<float>(curr_vertical_position   - last_vertical_position)   / 100.0) * (M_PI/180.0) * M_PI * 1.995;
	// 	auto hor_change = (static_cast<float>(curr_horizontal_position - last_horizontal_position) / 100.0) * (M_PI/180.0) * M_PI * 1.995;
		
	// 	printf("\\left(%f,%f\\right),\n", hor_change/imu_change, ver_change/imu_change);

	// 	last_imu = curr_imu;
	// 	last_vertical_position = curr_vertical_position;
	// 	last_horizontal_position = curr_horizontal_position;

	// 	pros::delay(30);
	// }
	// printf("\\right]");

	chassis.turnToHeading(-90, 5000);
	// chassis.moveToPoint(0, 24, 3000);
	// chassis.moveToPoint(0, 48, 5000);
	// chassis.turnToHeading(0, 3000);
	// chassis.turnToPoint(0, 0, 2000);
	// chassis.moveToPoint(0, 0, 3000);
	// chassis.turnToPoint(-24, 24, 3000);
	// chassis.moveToPoint(-24, 24, 3000);
	// chassis.turnToHeading(90, 3000);
	// chassis.moveToPoint(0, 0, 3000);
}
