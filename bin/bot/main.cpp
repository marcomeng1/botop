#include <BotOp/bot.h>
#include <KOMO/motionHelpers.h>

//===========================================================================

const char* usage = R"(usage: bot [-sim] [-confFile <file.g>] [-bot/useArm left|both] <actions>

Small utility to operate the robot (or its simulation with -sim).
Actions are executed in this order:
  -close   close the gripper(s)
  -open    open the gripper(s)
  -float   gravity compensation only, guide the arm by hand (until key press)
  -damp    compliant with damping (until key press)
  -hold    hold the current position (until key press)
  -up      move to home with the shoulder/elbow lifted
  -loop    move home, then along a loop path
  -home    move to the home pose
)";

void gripperAction(BotOp& bot, bool open){
  bot.hold(false, true);
  for(auto& g:{bot.gripperL, bot.gripperR}) if(g){ if(open) g->open(); else g->close(); }
  for(auto& g:{bot.gripperL, bot.gripperR}) if(g) while(!g->isDone()) rai::wait(.1);
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  if(argc<2 || rai::checkParameter<bool>("help")){
    cout <<usage <<endl;
    return 0;
  }

  //-- setup a configuration
  rai::Configuration C;
  if(rai::checkParameter<rai::String>("confFile")){
    C.addFile(rai::getParameter<rai::String>("confFile"));
  }else if(rai::getParameter<rai::String>("bot/useArm", "left")=="left"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  }else{
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));
  }

  BotOp bot(C, !rai::checkParameter<bool>("sim"));

  if(rai::checkParameter<bool>("close")) gripperAction(bot, false);
  if(rai::checkParameter<bool>("open")) gripperAction(bot, true);

  //-- control modes, each waits for a key press
  if(rai::checkParameter<bool>("float")){ bot.hold(true, false); bot.wait(C, true, false); }
  if(rai::checkParameter<bool>("damp")){ bot.hold(true, true); bot.wait(C, true, false); }
  if(rai::checkParameter<bool>("hold")){ bot.hold(false, true); bot.wait(C, true, false); }

  //-- motions
  if(rai::checkParameter<bool>("up")){
    arr q = bot.qHome;
    q(1) -= .5;
    if(q.N>7) q(8) -= .5;
    bot.moveTo(q, 1.);
    bot.wait(C, true, true);
  }

  if(rai::checkParameter<bool>("loop")){
    bot.moveTo(bot.qHome, 1.);
    bot.wait(C, true, true);

    C.setJointState(bot.qHome);
    arr path = getLoopPath(C);
    bot.move(path, {5.});
    bot.wait(C, true, true);
  }

  if(rai::checkParameter<bool>("home")){
    bot.moveTo(bot.qHome, 1.);
    bot.wait(C, true, true);
  }

  cout <<"bye bye" <<endl;
  return 0;
}
