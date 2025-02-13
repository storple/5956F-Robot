#pragma once
#include <string>

#include "robot/intake.h"
#include "robot/pneumatics.h"

namespace Robot
{
/**
 * @details We use Lemlib extensively for our documentation. You can find the documentation for Lemlib at
 * <A HREF="https://lemlib.github.io/lemlib/">Lemlib documentation</A>.
 * @brief The \ref Autonomous class contains classes and functions related to the robot's autonomous behavior.
 */
class Autonomous {
public:
	/**
	 * @brief Drives the robot autonomously.
	 *
	 * This function drives the robot autonomously based on the selected
	 * autonomous program. It takes a reference to a Puncher object and a boolean
	 * value indicating whether to use autonomous mode.
	 */
	void AutoDrive();
	void moveMotors(int power, int timeout);
	void WallSensorPID(float target_distance, int timeout, bool back);
	void ClampPID(float target_distance, int timeout, bool followthrough);
	void LimitSwitchIntake(int timeout);
	std::pair<float, float> distanceReset();
	float distanceResetX();
	float distanceResetY();

private:
	void RedNegative();
	void BlueNegative();
	void RedPositive();
	void BluePositive();
	void RedGoalRush();
	void BlueGoalRush();
	void RedRingRush();
	void BlueRingRush();
	void Test();
	void Skills();

	
};
} // namespace Robot