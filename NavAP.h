#pragma once


// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.			//
// ---------------------------------------------------- //

#include "UDPserver.h"
#include "RayBox.h"
#include "types.h"
#include <thread>
#include <string>

class NavAP
{
public:
  NavAP(int debug);
  void init();
  void NavAPMain();
  void getActiveIndex(int vesselIndex);
  bool isCollision;
  double currentThrust;
  double progressThrust;
  bool check_ping();
private:
  void setNavDestination(VECTOR3 *targetDest);
  void setBankSpeed(double value);
  void setPitchSpeed(double value);
  void setYawSpeed(double value);
  void setPitch(double pitch);
  void setRoll(double roll);
  void setDir(VECTOR3 *dir);
  double getDistance(VECTOR3 heading);
  double getRCSThrustByDelta(double deltaSpeed);
  double getAirspeedAngle();
  void getHeading(VECTOR3 *heading);
  double dot(VECTOR3 headingA, VECTOR3 headingB);
  double findAngleFromDot(double dot);
  double getRelativeHeadingAngle();
  void setupNewRay(RayBox *newRay, VECTOR3 *currentPosition);
  int activeIndex;
  VECTOR3 dest;
  VECTOR3 currentPos;
  VECTOR3 oldPos;
  UDPserver *serverConnect;
  int completedRCSOperations;
  double valuesRCS[3];
  double valuesDelta[3];
  std::string operation = "";
  std::string detail = "";
};
