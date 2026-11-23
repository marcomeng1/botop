#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>

/// IP address of Franka robot i (rai.cfg: Franka/ip0, Franka/ip1, ...); i = robotID for arms, whichRobot for grippers
inline rai::String franka_ipAddress(uint i){
  const char* defaults[2] = {"172.16.0.2", "172.17.0.2"};
  rai::String key = STRING("Franka/ip" <<i);
  return rai::getParameter<rai::String>(key.p, rai::String(i<2 ? defaults[i] : ""));
}

struct FrankaThread : rai::RobotAbstraction, Thread{
  FrankaThread(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state, uint robotID=0, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6})
      : RobotAbstraction(cmd, state), Thread("FrankaThread"){ init(robotID, _qIndices); }
  ~FrankaThread();

private:
  bool stop=false; //send end to libfranka
  bool requiresInitialization=true;  //waits in constructor until first contact/initialization
  int robotID=0;
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  arr friction;

  rai::String ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  void init(uint _robotID, const uintA& _qIndices);
  void step();
};
