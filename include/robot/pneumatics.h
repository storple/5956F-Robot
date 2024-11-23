#pragma once

namespace Robot {


class Pneumatics {
 public:

  bool state = false;
  
  void run();

  Pneumatics();

  bool detectMogo();
  void toggleLatch();
  void toggleDoinker();
  void toggleIntakeLift();

};
}  // namespace Robot