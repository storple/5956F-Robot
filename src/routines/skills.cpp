#include "robot/auton.h"
#include "main.h"

using namespace Robot;
using namespace Robot::Globals;

void Autonomous::Skills() 
{
	// /*
	chassis.setPose(0, -11.5, 0);
	subsystem.intake.toggle();
	pros::delay(300);
	chassis.moveToPoint(0, 0, 1000, {.maxSpeed=40});
	chassis.turnToPoint(-24, 0, 1000, {.forwards=false, .maxSpeed=70});
	ClampPID(10, 1750, false);
	subsystem.intake.toggle();
	pros::delay(300);
	subsystem.intake.toggle();
	chassis.turnToPoint(-48, 73, 750, {.maxSpeed=70});
	chassis.moveToPoint(-48, 73, 2000, {.maxSpeed=75}, false);
	chassis.turnToPoint(-58, 48, 750, {.maxSpeed=80});
	chassis.moveToPoint(-58, 48, 1500, {.maxSpeed=40});
	pros::delay(500);
	subsystem.arm.PID(3400, 750);
	chassis.moveToPose(-52, 52, -90, 850, {.forwards=false, .lead=0.75, .maxSpeed=80});
	chassis.turnToPoint(-72, 48, 500, {.maxSpeed=100}, false);
	// chassis.moveToPoint(-74, 48, 1500, {.maxSpeed=50});
	left_motors.move(60);
	right_motors.move(60);
	pros::delay(400);
	subsystem.intake.toggle();
	pros::delay(350);
	// subsystem.arm.PID(15400, 1100);
	ArmMotor.move(-20);
	pros::delay(400);
	ArmMotor.brake();
	pros::delay(100);
	chassis.moveToPoint(-48, 48, 1000, {.forwards=false, .maxSpeed=60});
	pros::delay(500);
	chassis.waitUntilDone();
	subsystem.arm.PID(500, 500);
	ArmMotor.move(-20);
	subsystem.intake.toggle();
	chassis.turnToPoint(-50, -16, 750, {.maxSpeed=80});
	chassis.moveToPoint(-50, -16, 2000, {.maxSpeed=60});
	// chassis.waitUntilDone();
	// chassis.setPose(-48, -7.5, 180);
	chassis.turnToPoint(-60, 6, 750, {.maxSpeed=80});
	chassis.moveToPoint(-60, 6, 1500, {.maxSpeed=70});
	// chassis.turnToPoint(-64, -12, 750, {.forwards=false, .maxSpeed=80});
	chassis.moveToPoint(-64, -12, 1100, {.forwards=false, .maxSpeed=80}, false);
	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch(); 
	pros::delay(500);
	subsystem.intake.toggle();

	chassis.moveToPose(0, 0, 90, 1500, {.maxSpeed=100});
	chassis.waitUntilDone();
	pros::delay(100);
	chassis.turnToHeading(-90, 1000, {.maxSpeed=50});
	chassis.waitUntilDone();
	ClampPID(10, 1750, false);

	// subsystem.pneumatics.toggleLatch();

	subsystem.intake.toggle();
	pros::delay(300);
	subsystem.intake.toggle();

	chassis.setPose(distanceResetX(), chassis.getPose().y, chassis.getPose().theta);
	chassis.turnToPoint(48, 73, 750, {.maxSpeed=80});
	chassis.moveToPoint(48, 73, 2000, {.maxSpeed=75}, false);
	chassis.turnToPoint(58, 48, 750, {.maxSpeed=80});
	chassis.moveToPoint(58, 48, 1500, {.maxSpeed=40});
	pros::delay(500);
	subsystem.arm.PID(3400, 750);
	chassis.moveToPose(52, 52, 90, 1100, {.forwards=false, .lead=0.75, .maxSpeed=80 });
	chassis.waitUntilDone();
	chassis.turnToPoint(72, 48, 500, {.maxSpeed=100}, false);
	left_motors.move(60);
	right_motors.move(60);
	pros::delay(400);
	subsystem.intake.toggle();
	pros::delay(350);
	ArmMotor.move(-20);
	pros::delay(400);
	ArmMotor.brake();
	pros::delay(100);
	chassis.moveToPoint(48, 48, 1000, {.forwards=false, .maxSpeed=60});
	pros::delay(500);
	chassis.waitUntilDone();
	subsystem.arm.PID(500, 500);
	ArmMotor.move(-20);
	subsystem.intake.toggle();
	chassis.turnToPoint(48, -16, 750, {.maxSpeed=80});
	chassis.moveToPoint(48, -16, 2000, {.maxSpeed=70});
	chassis.waitUntilDone();
	chassis.setPose(chassis.getPose().x, distanceResetY(),chassis.getPose().theta);
	pros::delay(50); 
	chassis.turnToPoint(60, 6, 750, {.maxSpeed=80});
	chassis.moveToPoint(60, 6, 1500, {.maxSpeed=60});
	// chassis.turnToPoint(64, -12, 750, {.forwards=false, .maxSpeed=80});
	chassis.moveToPoint(64, -12, 900, {.forwards=false, .maxSpeed=80});
	chassis.waitUntilDone();
	subsystem.intake.toggle();
	subsystem.pneumatics.toggleLatch();
	pros::delay(250);

    // second half
	chassis.moveToPoint(48, 48, 1500, {.maxSpeed=90});

	chassis.turnToPoint(24, 76, 500, {.maxSpeed=80});
	chassis.moveToPoint(24, 76, 1500, {.maxSpeed=80});
    pros::delay(350);
    subsystem.intake.toggle();
    LimitSwitchIntake(600);
    
    chassis.turnToPoint(0, 108, 1000, {.forwards=false, .maxSpeed=80});
    ClampPID(10,1750,false);
    pros::delay(250);
    subsystem.intake.toggle();
	pros::delay(500);
    chassis.turnToPoint(-24, 72, 750, {.maxSpeed=80});
    chassis.moveToPoint(-24, 72, 1000, {.maxSpeed=80});

	chassis.turnToPoint(-46, 72, 750, {.maxSpeed=80});
	chassis.moveToPoint(-46, 72, 1000, {.maxSpeed=80});
	chassis.turnToHeading(-90, 400, {.maxSpeed=60}, false);
	chassis.setPose((4.9+((wall_sensor.get())/25.4)) - 70.4, chassis.getPose().y, chassis.getPose().theta);
	controller.print(0, 0, "%f, %f",chassis.getPose().x, chassis.getPose().y);
	pros::delay(100);
	chassis.turnToHeading(0, 750, {.maxSpeed=80});
	chassis.moveToPoint(-48, 108, 1500, {.maxSpeed=85});
	subsystem.pneumatics.toggleLeftDoinker();
    // chassis.turnToPoint(2, 46, 750, {}, false);
    // chassis.moveToPoint(2, 46, 1000, {.maxSpeed=80});
	// pros::delay(100);
    // subsystem.intake.toggle();
    // pros::delay(850);
    // subsystem.intake.toggle();
    // pros::delay(500);
    // subsystem.intake.toggle();
	// subsystem.pneumatics.toggleLeftDoinker();
    // chassis.moveToPoint(-49, 95, 1500, {.maxSpeed=100});
	// pros::delay(1000);
	// subsystem.intake.toggle();
	// chassis.waitUntilDone();


	chassis.turnToPoint(-60, 112, 600, {.forwards=false, .direction=AngularDirection::CW_CLOCKWISE});
	pros::delay(300);
	subsystem.pneumatics.toggleLeftDoinker();
	chassis.moveToPoint(-60, 112, 1250, {.forwards=false, .maxSpeed=80}, false);
    subsystem.intake.toggle();
    subsystem.pneumatics.toggleLatch();
    chassis.moveToPoint(-24, 86, 1000);
    chassis.turnToPoint(0, 103, 500);
	chassis.moveToPoint(0, 103, 750);
    chassis.moveToPoint(60, 112, 2200, {}, false);
	subsystem.arm.PID(15400, 500);
	chassis.moveToPose(30, 78, 45, 1150, {.forwards = false,.minSpeed=70});
	chassis.moveToPoint(0, 48, 5000, {.forwards=false, .maxSpeed=70});
}

// old skills
// void Autonomous::Skills() 
// {
// 	chassis.setPose(0, -11.5, 0);
// 	subsystem.intake.toggle();
// 	pros::delay(750);
// 	chassis.moveToPoint(0, 0, 1000, {.maxSpeed=40});
// 	chassis.turnToPoint(-24, 0, 1000, {.forwards=false, .maxSpeed=70});
// 	ClampPID(13, 2500, false);
// 	// subsystem.pneumatics.toggleLatch();
// 	subsystem.intake.toggle();
// 	pros::delay(500);
// 	subsystem.intake.toggle();
// 	chassis.turnToPoint(-48, 72, 1000);
// 	chassis.moveToPoint(-48, 72, 2000, {.maxSpeed=70});
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	chassis.turnToPoint(-56, 48, 1000);
// 	chassis.moveToPoint(-56, 48, 1500, {.maxSpeed=40});
// 	pros::delay(500);
// 	subsystem.arm.PID(4250, 750);
// 	chassis.moveToPose(-52, 52, -90, 1250, {.forwards=false, .lead=0.75, .maxSpeed=60});
// 	chassis.waitUntilDone();
// 	chassis.turnToPoint(-66, 48, 750);
// 	subsystem.intake.toggle();
// 	chassis.moveToPoint(-66, 48, 2000, {.maxSpeed=30});
// 	pros::delay(750);
// 	subsystem.arm.PID(15400, 1500);
// 	pros::delay(200);
// 	chassis.moveToPoint(-47, 48, 1000, {.forwards=false, .maxSpeed=60});
// 	pros::delay(500);
// 	chassis.waitUntilDone();
// 	subsystem.arm.PID(1400, 500);
// 	subsystem.intake.toggle();
// 	chassis.turnToPoint(-50, -12, 1000, {.maxSpeed=80});
// 	chassis.moveToPoint(-50, -12, 2000, {.maxSpeed=50});
// 	// chassis.waitUntilDone();
// 	// chassis.setPose(-48, -7.5, 180);
// 	chassis.turnToPoint(-60, 2, 1000, {.maxSpeed=80});
// 	chassis.moveToPoint(-60, 2, 1500, {.maxSpeed=60});
// 	chassis.turnToPoint(-64, -12, 1000, {.forwards=false, .maxSpeed=80});
// 	chassis.moveToPoint(-64, -12, 750, {.forwards=false, .maxSpeed=50});
// 	chassis.waitUntilDone();
// 	subsystem.intake.toggle();
// 	subsystem.pneumatics.toggleLatch();
// 	pros::delay(500);
// 	subsystem.intake.toggle();

// 	chassis.moveToPoint(0, 0, 2000, {.maxSpeed=70});
// 	chassis.waitUntilDone();
// 	pros::delay(100);
// 	chassis.turnToHeading(-90, 1000, {.maxSpeed=50});
// 	chassis.waitUntilDone();
// 	ClampPID(13, 1500, false);
// 	// subsystem.pneumatics.toggleLatch();

// 	subsystem.intake.toggle();
// 	pros::delay(500);
// 	subsystem.intake.toggle();
// 	chassis.turnToPoint(48, 72, 1000);
// 	chassis.moveToPoint(48, 72, 2000, {.maxSpeed=70});
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	chassis.turnToPoint(56, 48, 1000);
// 	chassis.moveToPoint(56, 48, 1500, {.maxSpeed=40});
// 	pros::delay(500);
// 	subsystem.arm.PID(4250, 750);
// 	chassis.moveToPose(52, 52, 90, 1250, {.forwards=false, .lead=0.75, .maxSpeed=60 });
// 	chassis.waitUntilDone();
// 	chassis.turnToPoint(66, 48, 750);
// 	subsystem.intake.toggle();
// 	chassis.moveToPoint(66, 48, 2000, {.maxSpeed=30});
// 	pros::delay(750);
// 	subsystem.arm.PID(15400, 1500);
// 	pros::delay(200);
// 	chassis.moveToPoint(47, 48, 1000, {.forwards=false, .maxSpeed=60});
// 	pros::delay(500);
// 	chassis.waitUntilDone();
// 	subsystem.arm.PID(1400, 500);
// 	subsystem.intake.toggle();
// 	chassis.turnToPoint(48, 0, 1000, {.maxSpeed=80});
// 	chassis.moveToPoint(48, -14, 2000, {.maxSpeed=50});
// 	// chassis.waitUntilDone();
// 	// chassis.setPose(48, -7.5, 180);
// 	chassis.turnToPoint(60, 2, 1000, {.maxSpeed=80});
// 	chassis.moveToPoint(60, 2, 1500, {.maxSpeed=60});
// 	chassis.turnToPoint(64, -12, 1000, {.forwards=false, .maxSpeed=80});
// 	chassis.moveToPoint(64, -12, 750, {.forwards=false, .maxSpeed=50});
// 	chassis.waitUntilDone();
// 	subsystem.intake.toggle();
// 	subsystem.pneumatics.toggleLatch();
// 	pros::delay(500);
// 	// subsystem.intake.toggle();	

// 	chassis.moveToPoint(55, 48, 1000, {.earlyExitRange=36});
// 	chassis.moveToPoint(48, 72, 1500, {.earlyExitRange=36});
// 	chassis.moveToPoint(-24, 112, 2000, {.minSpeed=50, .earlyExitRange=36});
// 	chassis.moveToPoint(-56, 114, 2500, {.minSpeed=50});
// 	chassis.moveToPoint(0, 100, 2000, {.forwards=false});
// 	chassis.moveToPoint(56, 114, 3000, {.forwards = false});

// 	chassis.waitUntilDone();
// 	subsystem.arm.PID(15400, 500);
// 	chassis.moveToPoint(30, 78, 1000, {.minSpeed=50});
// 	chassis.turnToHeading(45, 1000);
// 	chassis.moveToPoint(0, 48, 5000, {.forwards=false, .maxSpeed=60});
// 	// 24,72
// 	// turn to 45
// 	// back up slow
// }


