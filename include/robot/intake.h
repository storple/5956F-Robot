#pragma once

namespace Robot
{

/**
 * @brief The Intake class represents a robot intake system.
 */
class Intake {
public:

	bool currentlyOn;
	bool playingRed;
	
	/**
	 * @brief Runs the main function of the intake system.
	 *
	 * Takes optional user input to control the direction of the intake system in
	 * autonomous.
	 */
	bool detectRing();
	bool detectBadColor();
	void run();
	void run(int msec);
	void toggle();

	/**
	 * @brief Outakes alliance rings into latched mobile goal and ejects enemy rings.
	 */
	void score();

	Intake();

private:
	bool elevated;

	/**
	 * @brief blue is false, red is true.
	 */
	bool alliance_color;

};
}