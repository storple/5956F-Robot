#pragma once

namespace Robot {

/**
 * @brief The Arm class represents an arm mechanism.
 */
class Arm {


  public:

    int currentState = -1;
    bool autoMode = false;
    bool pressed = true;

    void PID(float target);

    /**
     * @brief Runs arm mechanism.
     */
    void run();

    Arm();
  

};
}  // namespace Robot