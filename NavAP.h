#pragma once


// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.			//
// ---------------------------------------------------- //

#include "UDPserver.h"
#include "types.h"
#include <thread>

class NavAP
{
public:
  NavAP(int debug);
  void init();
  VECTOR3 setNavDestination();
  void NavAPMain();
  void getActiveIndex(int vesselIndex);
  bool isCollision;
  double currentThrust;
  double progressThrust;
  bool check_ping();
private:
  void setBankSpeed(double value);
  void setPitchSpeed(double value);
  void setYawSpeed(double value);
  double setPitch(double pitch);
  double setRoll(double roll);
  double setDir(double dir);
  void getDir(VECTOR3 dir);
  double getDistance(VECTOR3 heading);
  double getRCSThrustByDelta(double deltaSpeed);
  double getAirspeedAngle();
  int activeIndex;
  VECTOR3 dest;
  VECTOR3 currentPos;
  VECTOR3 oldPos;
  UDPserver *serverConnect;
  int completedRCSOperations;
  double valuesRCS[3];
  double valuesDelta[3];
};
