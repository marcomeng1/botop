#include "Move_IK.h"

#include <Optim/NLP_Solver.h>

Move_IK::Move_IK(BotOp& _bot, rai::Configuration& _C, int _askForOK) : bot(_bot), C(_C), askForOK(_askForOK){
  komo.setTiming(1., 1, 1., 0);
  komo.setConfig(C, true);
  komo.addControlObjective({}, 0, 1e-1);
}

bool Move_IK::go(){
  rai::NLP_Solver sol;
  sol.setProblem(komo.nlp());
  sol.opt->set_stopTolerance(1e-4);
  auto ret = sol.solve();
  cout <<*ret <<endl;
  if(!ret->feasible){
    return false;
  }else{
    qT = komo.getPath_qOrg()[-1];
    C.setJointState(qT);
    if(askForOK && C.view(true, "Move_IK\ngo?")=='q') return false;

    bot.moveTo(qT, 2.);
    for(;bot.getTimeToEnd()>0.;) bot.sync(C);
  }
  return true;
}
