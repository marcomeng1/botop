#pragma once

#include <Control/CtrlMsgs.h>
#include <Kin/kin.h>

#include <functional>
#include <vector>

/* Registry of hardware drivers. BotOp starts a driver when the model contains its frame:
   for sided drivers the frame is looked up with prefix "l_" and "r_" (e.g. "l_panda_base"),
   otherwise by its full name (e.g. "omnibase_world").
   A driver registers itself in its own .cpp, e.g.

     static botop::RegisterArm _reg("panda_base", [](auto& cmd, auto& state, uint robotID, const rai::Configuration& C, char side){
       return std::make_shared<FrankaThread>(cmd, state, robotID, franka_getJointIndices(C, side)); });
*/

namespace botop {

/// side is 'l', 'r', or 0 for drivers without side; robotID counts the arms created by this driver (0, 1, ...)
using ArmFactory = std::function<std::shared_ptr<rai::RobotAbstraction>(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state,
                                                                        uint robotID, const rai::Configuration& C, char side)>;
/// whichRobot is 0 for the left and 1 for the right gripper
using GripperFactory = std::function<std::shared_ptr<rai::GripperAbstraction>(uint whichRobot)>;

struct ArmDriver { rai::String frame; bool sided; ArmFactory create; };
struct GripperDriver { rai::String frame; GripperFactory create; };

std::vector<ArmDriver>& armDrivers();
std::vector<GripperDriver>& gripperDrivers();

struct RegisterArm {
  RegisterArm(const char* frame, ArmFactory create, bool sided=true){ armDrivers().push_back({frame, sided, create}); }
};
struct RegisterGripper {
  RegisterGripper(const char* frame, GripperFactory create){ gripperDrivers().push_back({frame, create}); }
};

} //namespace
