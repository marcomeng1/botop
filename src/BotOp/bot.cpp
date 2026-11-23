#include "bot.h"

#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <KOMO/pathTools.h>
#include <Control/timingOpt.h>
#include <Optim/NLP_Solver.h>
#include <Gui/opengl.h>
#include <Geo/depth2PointCloud.h>

#include "drivers.h"
#include <Sim/simulation.h>
#include <OptiTrack/optitrack.h>
#include <RealSense/RealSenseThread.h>
#include <Perception/ArucoThread.h>

#include <Audio/audio.h>

//===========================================================================

BotOp::BotOp(rai::Configuration& C, bool useRealRobot){
  //-- launch arm(s) & gripper(s)
  bool useGripper = rai::getParameter<bool>("bot/useGripper", true);
  bool blockRealRobot = rai::getParameter<bool>("bot/blockRealRobot", false);
  forceRealCamera = rai::getParameter<bool>("bot/forceRealCamera", false);

  C.ensure_indexedJoints();
  qHome = C.getJointState();
  state.set()->initZero(qHome.N);

  if(blockRealRobot && useRealRobot){
    LOG(0) <<"-- blocking useRealRobot -- ";
    useRealRobot=false;
  }

  //-- launch robots & grippers: drivers are chosen by frame names in the model (see drivers.h)
  if(useRealRobot && useGripper){
    for(char side : {'l', 'r'}){
      std::shared_ptr<rai::GripperAbstraction>& gripper = (side=='l' ? gripperL : gripperR);
      for(botop::GripperDriver& d : botop::gripperDrivers()){
        rai::String frame = STRING(side <<'_' <<d.frame);
        if(!C.getFrame(frame, false)) continue;
        if(gripper){ LOG(-1) <<"more than one gripper driver matches side '" <<side <<"' -- ignoring '" <<frame <<"'"; continue; }
        LOG(0) <<"CONNECTING TO GRIPPER '" <<frame <<"'";
        try{
          gripper = d.create(side=='l' ? 0 : 1);
        } catch(const std::exception& ex) {
          LOG(-1) <<"Starting the gripper '" <<frame <<"' failed! Error msg: " <<ex.what();
        }
      }
    }
    if(!gripperL && !gripperR) LOG(0) <<"WARNING: no grippers created";
  }

  if(useRealRobot){
    //sided drivers (l_/r_) first, so that drivers without side take the remaining slot
    for(bool sided : {true, false}){
      for(botop::ArmDriver& d : botop::armDrivers()){
        if(d.sided!=sided) continue;
        uint robotID=0;
        for(char side : (sided ? std::vector<char>{'l', 'r'} : std::vector<char>{0})){
          rai::String frame = sided ? STRING(side <<'_' <<d.frame) : d.frame;
          if(!C.getFrame(frame, false)) continue;
          std::shared_ptr<rai::RobotAbstraction>& robot = (side=='r' || (!side && robotL)) ? robotR : robotL;
          if(robot){ LOG(-1) <<"no free robot slot for '" <<frame <<"' -- ignoring"; continue; }
          LOG(0) <<"CONNECTING TO ROBOT '" <<frame <<"'";
          try{
            robot = d.create(cmd, state, robotID++, C, side);
          } catch(const std::exception& ex) {
            LOG(-1) <<"Starting the robot '" <<frame <<"' failed! Error msg: " <<ex.what();
          } catch(...) {
            LOG(-1) <<"Starting the robot '" <<frame <<"' failed! Error msg: " <<rai::errString();
          }
        }
      }
    }
    if(!robotL && !robotR) LOG(0) <<"starting botop without robots";
    else C.setJointState(get_q());

  }else{
    simthread = make_shared<BotThreadedSim>(C, cmd, state);
    robotL = simthread;
    if(useGripper) gripperL = make_shared<GripperSim>(simthread, "l_gripper");
  }

  startRealTime = rai::realTime();

  //-- initialize the control reference
  hold(false, true);

  //-- launch OptiTrack
  if(rai::getParameter<bool>("bot/useOptitrack", false)){
    LOG(0) <<"OPENING OPTITRACK";
    if(!useRealRobot) LOG(-1) <<"useOptitrack with real:false -- that's usually wrong!";
    optitrack = make_shared<rai::OptiTrack>();
    optitrack->pull(C);
  }

  //-- launch Audio/Sound
  if(rai::getParameter<bool>("bot/useAudio", false)){
    LOG(0) <<"OPENING SOUND";
    audio = make_shared<rai::Sound>();
  }

  C.view(false, STRING("time: 0"));
  C.gl().setTitle("BotOp associated Configuration");
}

BotOp::~BotOp(){
  LOG(0) <<"shutting down BotOp...";
  for(auto& ar:aruco_threads) ar.reset();
  for(auto& cam:cameras) cam.reset();
  if(simthread) simthread.reset();
  gripperL.reset();
  gripperR.reset();
  robotL.reset();
  robotR.reset();
}

double BotOp::get_t(){
  return state.get()->ctrlTime;
}

void BotOp::getState(arr& q_real, arr& qDot_real, double& ctrlTime){
  auto stateGet = state.get();
  q_real = stateGet->q;
  qDot_real = stateGet->qDot;
  ctrlTime = stateGet->ctrlTime;
}

void BotOp::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  auto cmdGet = cmd.get();
  CHECK(cmdGet->ref, "reference not initialized yet!");
  cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, qDot_real, ctrlTime);
}

arr BotOp::get_q() {
  return state.get()->q;
}

arr BotOp::get_qDot() {
  return state.get()->qDot;
}

double BotOp::getTimeToEnd(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp){
    LOG(-1) <<"can't get timeToEnd for non-spline mode";
    return 0.;
  }
  double ctrlTime = get_t();
  return sp->getEndTime() - ctrlTime;
}

arr BotOp::getEndPoint(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp) return get_q();
  return sp->getEndPoint();
}

arr BotOp::get_tauExternal(){
  arr tau;
  {
    auto stateSet = state.set();
    tau = stateSet->tauExternalIntegral;
    tau /= double(stateSet->tauExternalCount);
    stateSet->tauExternalIntegral.setZero();
    stateSet->tauExternalCount=0;
  }
  return tau;
}

int BotOp::sync(rai::Configuration& C, double waitTime, rai::String viewMsg){
  //update q state
  C.setJointState(state.get()->q);

  //update optitrack state
  if(optitrack) optitrack->pull(C);

  if(cameras.N){
    auto& V = *C.get_viewer();
    for(uint i=0;i<cameras.N;i++){
      byteA rgb = cameras(i)->image.get();
      if(rgb.N) V.setQuad(i, rgb, i*.3, .8, .2);
    }
  }

  //update sim state
  if(simthread) simthread->pullDynamicStates(C);

  //gui
  if(rai::getParameter<bool>("bot/raiseWindow",false)) C.get_viewer()->raiseWindow();
  double ctrlTime = get_t();
  keypressed = C.view(false, STRING("BotOp sync ctrl time: "<<ctrlTime <<" (=" <<int(100.*ctrlTime/(rai::realTime()-startRealTime)) <<"% real time)\n" <<viewMsg));
  if(keypressed) C.get_viewer()->_resetPressedKey();
//  if(keypressed==13) return false;
//  if(keypressed=='q' || keypressed==27) return false;
//  auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
//  if(sp && ctrlTime>sp->getEndTime()) return false;
  if(!keypressed && waitTime>0.) rai::wait(waitTime);
  return keypressed;
}

int BotOp::wait(rai::Configuration& C, bool forKeyPressed, bool forTimeToEnd, bool forGripper, double syncFrequency){
  C.get_viewer()->raiseWindow();
  C.get_viewer()->_resetPressedKey();
  sync(C);
  for(;;){
    //if(keypressed=='q') return keypressed;
    if(forKeyPressed && keypressed) return keypressed;
    if(forTimeToEnd && getTimeToEnd()<=0.) return keypressed;
    if(forGripper && gripperDone(rai::_left)) return 'g';
    if(!rai::getInteractivity() && !forTimeToEnd && forKeyPressed) return ' ';
    sync(C, syncFrequency);
  }
}

std::shared_ptr<rai::BSplineCtrlReference> BotOp::getSplineRef(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp){
    setReference<rai::BSplineCtrlReference>();
    sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
    CHECK(sp, "this is not a spline reference!")
  }
  return sp;
}

void BotOp::move(const arr& path, const arr& times, bool overwrite, double overwriteCtrlTime){
  arr _times=times;

  CHECK(_times.N, "move needs some times specified - use moveAutoTimed as alternative");

  //-- if times.N != path.d0, fill in times
  if(_times.N==1 && path.d0>1){ //divide total time in grid
    _times = range(0., times.scalar(), path.d0-1);
    _times += _times(1);
  }
  if(_times.N){ //times are fully specified
    CHECK_EQ(_times.N, path.d0, "");
  }

  if(overwrite){
    CHECK(overwriteCtrlTime>0., "overwrite -> need to give a cut-time (e.g. start or MPC cycle, or just get_t())");
    //LOG(1) <<"overwrite: " <<ctrlTime <<" - " <<_times;
    if(times.first()>0.){
      getSplineRef()->overwriteSmooth(path, /*vels,*/ _times, overwriteCtrlTime);
    }else{
      getSplineRef()->overwriteHard(path, /*vels,*/ _times, overwriteCtrlTime);
    }
  }else{
    //LOG(1) <<"append: " <<ctrlTime <<" - " <<_times;
    getSplineRef()->append(path, /*vels,*/ _times, get_t());
  }
}

void BotOp::moveAutoTimed(const arr& path, double maxVel, double maxAcc){
  CHECK_GE(path.d0, 16, "this only works for smooth paths!");
  double D = getMinDuration(path, maxVel, maxAcc);
  arr times = range(0., D, path.d0-1);
  times += times(1);
  move(path, times);
}

void BotOp::moveTo(const arr& q_target, double timeCost, bool overwrite){
  arr q, qDot;
  double ctrlTime=0.;
  if(overwrite){
    getState(q, qDot, ctrlTime);
  }else{
    q = getEndPoint();
    qDot.resize(q.N).setZero();
  }
  double dist = length(q-q_target) +1e-4; //plus eps for numerical robustness
  double vel = scalarProduct(qDot, q_target-q)/dist;
  double T = (sqrt(6.*timeCost*dist+vel*vel) - vel)/timeCost;
  //cout <<q <<qDot <<q_target <<dist <<' ' <<vel <<' ' <<T <<' ' <<ctrlTime <<endl;
  if(dist<1e-4 || T<.1) T=.1;
  if(overwrite){
    move(~q_target, {T}, true, ctrlTime);
  }else{
    move(~q_target, {T}, false);
  }
}

StepObservation BotOp::stepObservation(){
  StepObservation obs;
  getState(obs.qpos, obs.qvel, obs.ctrlTime);
  getReference(obs.qref, NoArr, NoArr, obs.qpos, obs.qvel, obs.ctrlTime);
  return obs;
}

void BotOp::stepAction(const arr& delta, const StepObservation& obs, double lambda, double maxAccel, double xi){
  arr x0 = obs.qref; //reference!!
  arr v0 = obs.qvel; //real!!
  arr accel = (delta-2.*xi*lambda*v0)/(lambda*lambda); // accel = 2.*coeff (in comparison to python SecondOrderPolyRef)!
  double a = absMax(accel);
  if(a>maxAccel) accel *= maxAccel/a;
  double time = obs.ctrlTime;

  {
    auto splineSet = getSplineRef()->spline.set(); //mutexed for threading
    splineSet->setConstAccelPiece(time, x0, v0, accel, lambda);
  }
}

void BotOp::setControllerWriteData(int _writeData){
  if(robotL) robotL->writeData=_writeData;
  if(robotR) robotR->writeData=_writeData;
}

void BotOp::setCompliance(const arr& J, double compliance){
  CHECK_LE(compliance, 1., "");
  CHECK_GE(compliance, 0., "");
  if(!J.N || !compliance){
    LOG(0) <<"clearing compliance";
    cmd.set()->P_compliance.clear();
    return;
  }


  arr U, d, V;
  svd(U, d, V, J, false);
  CHECK_EQ(d.N, J.d0, "");
  for(uint i=0;i<d.N;i++) CHECK_GE(fabs(d(i)), 1e-3, "singular Jacobian?");

  arr P = eye(J.d1);
  P -= compliance * (V*~V);

  cmd.set()->P_compliance = P;
}

void BotOp::gripperMove(rai::ArgWord leftRight, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->open(width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->open(width, speed); }
}

void BotOp::gripperClose(rai::ArgWord leftRight, double force, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->close(force, width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->close(force, width, speed); }
}

void BotOp::gripperCloseGrasp(rai::ArgWord leftRight, const char* objName, double force, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->closeGrasp(objName, force, width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->closeGrasp(objName, force, width, speed); }
}

double BotOp::getGripperPos(rai::ArgWord leftRight){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else return gripperL->pos(); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else return gripperR->pos(); }
  return 0;
}

bool BotOp::gripperDone(rai::ArgWord leftRight){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else return gripperL->isDone(); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else return gripperR->isDone(); }
  return true;
}

void BotOp::launch_arucos(){
  for(uint k=0;k<cameras.N;k++){
    aruco_threads.append(make_shared<rai::ArucoThread>(k, cameras(k)->image));
  }
}

std::shared_ptr<rai::CameraAbstraction>& BotOp::getCamera(const char* sensor){
  for(std::shared_ptr<rai::CameraAbstraction>& cam:cameras){
    if(cam->camera_name==sensor) return cam;
  }
  if(simthread && !forceRealCamera){
    cameras.append( make_shared<CameraSimThread>(simthread, sensor) );
  }else{
    int cameraID = -1;
    str name = sensor;
    if(name.startsWith("RealSense_")){
      name.sub(10,0) >>cameraID;
    }
    cameras.append( make_shared<RealSenseThread>(sensor, cameraID) );
  }
  return cameras(-1);
}

void BotOp::getImageAndDepth(byteA& image, floatA& depth, const char* sensor){
  auto cam = getCamera(sensor);
  image = cam->image.get();
  depth = cam->depth.get(); //getImageAndDepth(image, depth);
}

arr BotOp::getCameraFxycxy(const char* sensor){
  auto cam = getCamera(sensor);
  return cam->getFxycxy();
}

void BotOp::getImageDepthPcl(byteA& image, floatA& depth, arr& points, const char* sensor, bool globalCoordinates){
  auto cam = getCamera(sensor);
  image = cam->image.get();
  depth = cam->depth.get(); //getImageAndDepth(image, depth);
  depthData2pointCloud(points, depth, cam->getFxycxy());
  if(globalCoordinates){
    rai::Transformation pose=cam->getPose();
    if(!pose.isZero()) pose.applyOnPointArray(points);
  }
}

void BotOp::home(rai::Configuration& C){
  C.get_viewer()->raiseWindow();
  moveTo(qHome, 1., true);
  wait(C);
}

void BotOp::stop(rai::Configuration& C){
  C.get_viewer()->raiseWindow();
  moveTo(get_q(), .01, true);
  wait(C);
}

void BotOp::hold(bool floating, bool damping){
  auto zref = std::dynamic_pointer_cast<ZeroReference>(ref);
  if(!zref){
    setReference<ZeroReference>();
    zref = std::dynamic_pointer_cast<ZeroReference>(ref);
    CHECK(zref, "this is not a spline reference!")
  }
  if(floating){
    zref->setPositionReference({});
    if(damping){
      zref->setVelocityReference({0.}); //{0.}: have a Kd with zero vel ref;
    }else{
      zref->setVelocityReference({}); //{}: have no Kd term at all; {1.} have a Kd term with velRef=velTrue (and friction compensation!)
    }
  }else{
    arr q = get_q();
    zref->setPositionReference(q);
    zref->setVelocityReference({0.});
  }
}

void BotOp::sound(int noteRelToC, float a, float decay){
  if(audio){
    audio->addNote(noteRelToC, a, decay);
  }
}

void BotOp::attach(str from, str to) {
  if(!simthread){
    LOG(-1) <<"attach only works in sim";
  }else{
    simthread->attach(from, to);
  }
}

void BotOp::detach(str from, str to){
  if(!simthread){
    LOG(-1) <<"attach only works in sim";
  }else{
    simthread->detach(from, to);
  }
}

void BotOp::cheat_setFramePose(str name, const arr& pose){
  if(!simthread){
    LOG(-1) <<"cheat only works in sim";
  }else{
    rai::Frame *f = simthread->simConfig.getFrame(name);
    if(pose.N) f->setPose(pose);
    simthread->sim->pushConfigToSim();
  }
}

//===========================================================================

void ZeroReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  {
    arr pos = position_ref.get()();
    if(pos.N) q_ref = pos;
    else q_ref.clear(); // = q_real;  //->no position gains at all
  }
  {
    arr vel = velocity_ref.get()();
    if(vel.N==1){
      double a = vel.scalar();
      CHECK(a>=0. && a<=1., "");
      qDot_ref = a * qDot_real; //[0] -> zero vel reference -> damping
    }
    else if(vel.N) qDot_ref = vel;
    else qDot_ref.clear(); //.clear();  //[] -> no damping at all! (and also no friction compensation based on reference qDot)
  }
  qDDot_ref.clear(); //[] -> no acc at all
}

