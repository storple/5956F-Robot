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


private:
	void Auton1();
	void Auton2();
	void Auton3();
	void Auton4();
	void Skills();

	
};
} // namespace Robot