#pragma once

namespace Robot {

/**
 * @brief The Arm class represents an arm mechanism.
 */
class Arm {
  public:
    /**
     * @brief Default constructor for Arm.
     */
    Arm();

    /**
     * @brief Runs arm mechanism.
     */
    void run();

    /**
     * @brief Updates the PID control for the arm.
     */
    void update();

    /**
     * @brief Sets a new target for the arm PID control.
     * 
     * @param target The desired target angle.
     */
    void setArmState(int mode);
    void PID(float target_angle, int timeout);

  private:
    int currentState = -1;  // Current arm state
    bool autoMode = false;  // Whether the arm is in automatic mode
    bool pressed = true;    // Whether a button press is detected

    float target_angle = 1400;  // Target angle for the arm
    bool pid_active = false;    // Whether the PID control is active

    /**
     * @brief Private helper function for PID control.
     * 
     * @param target_angle The desired target angle.
     */
};

}  // namespace Robot
