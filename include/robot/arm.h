#pragma once

namespace Robot {

/**
 * @brief The Arm class represents an arm mechanism.
 */
class Arm {


  public:

    int currentState;
    float current;    // Current position from sensor
    float error;
    float lastError;
    float integral;
    float derivative;

    // PID constants
    float kP;
    float kI;
    float kD;

    void PID(float target);

    /**
     * @brief Runs arm mechanism.
     */
    void run();

    Arm();
  

};
}  // namespace Robot