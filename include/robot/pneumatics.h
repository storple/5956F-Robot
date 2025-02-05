#pragma once

namespace Robot {


class Pneumatics {
 public:

  void run();

  Pneumatics();

  bool detectMogo();
  void toggleLatch();
  void toggleDoinker();
  void toggleIntakeLift();

 private:
  bool charged = false;
};
}  // namespace Robot