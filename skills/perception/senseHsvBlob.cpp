#include "senseHsvBlob.h"

#include <Gui/opengl.h>
#include <Perception/cvTools.h>
#include <Geo/depth2PointCloud.h>
#include <Kin/frame.h>

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C, const char* camName, const char* blobName, const arr& hsvFilter, const arr& Pinv, int verbose){
  OpenGL disp;
  byteA img;
  floatA depth;
  bot.getImageAndDepth(img, depth, camName);
  arr u = getHsvBlobImageCoords(img, depth, hsvFilter, verbose-1);
  if(verbose>1) disp.watchImage(img, false);
  if(verbose>0) LOG(0) <<"dot in image coords: " <<u;
  if(!u.N) return false;

  arr x;
  if(Pinv.N){
    makeHomogeneousImageCoordinate(u);
    x = Pinv*u;
  }else{
    x = u;
    depthData2point(x, bot.getCameraFxycxy(camName));
  }
  if(verbose>0) LOG(0)  <<"dot in cam coords: " <<x;
  C[camName]->get_X().applyOnPoint(x);
  if(verbose>0) LOG(0)  <<"dot in world coords: " <<x;

  C[blobName]->setPosition(x);
  if(verbose>0 && C.view(true, "sense_HsvBlob\ngo?")=='q') return false;

  return true;
}
