#include "drivers.h"

namespace botop {

std::vector<ArmDriver>& armDrivers(){
  static std::vector<ArmDriver> drivers;
  return drivers;
}

std::vector<GripperDriver>& gripperDrivers(){
  static std::vector<GripperDriver> drivers;
  return drivers;
}

} //namespace
