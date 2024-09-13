#pragma once

namespace Robot {

/**
 * @brief The Arm class represents an arm mechanism.
 */
class Arm {
 public:
  /**
   * @brief Runs arm mechanism.
   */
  void run();

  Arm();

  /**
   * @brief Toggles arm state.
   */
  void toggle();

};
}  // namespace Robot