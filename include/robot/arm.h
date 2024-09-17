#pragma once

namespace Robot {

/**
 * @brief The Arm class represents an arm mechanism.
 */
class Arm {


  public:

    int currentState;

    void PID(float target);

    /**
     * @brief Runs arm mechanism.
     */
    void run();

    Arm();
  

};
}  // namespace Robot