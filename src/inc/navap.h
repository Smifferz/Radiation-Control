#ifndef NAVAP_H
#define NAVAP_H

// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.			//
// ---------------------------------------------------- //

#include "udpserver.h"
#include "raybox.h"
#include "types.h"
#include <thread>
#include <string>


/**
 * The NavAP class controls the navigation autopilot for remote navigation
 * @brief The class that performs navigation
 */
class NavAP
{
public:
  NavAP(std::string ip, int debug, std::string file);
  void init();
  void NavAPMain();
  void getActiveIndex(int vesselIndex);
  bool isCollision;
  double currentThrust;
  double progressThrust;
  bool check_ping();
private:
  void setNavDestination(v3 targetDest);
  void getCurrentRotVel(v3 *currentRotVel);
  void setBankSpeed(double value);
  void setPitchSpeed(double value);
  void setYawSpeed(double value);
  void setPitch(double pitch);
  double getPitch();
  void setRoll(double roll);
  double getBank();
  void setYaw(double yaw);
  double getYaw();
  void setDir(v3 *dir, bool normal);
  double getDistance(v3 heading);
  double getRCSThrustByDelta(double deltaSpeed);
  double getAirspeedAngle();
  void getHeading(v3 *heading, bool normal);
  double dot(v3 headingA, v3 headingB);
  double findAngleFromDot(double dot);
  double getRelativeHeadingAngle();
  double getComponentAngle(double adjacent, double hypotenuse);
  void normalise(v3* normalVector, double vectorLength);
  void setupNewRay(RayBox *newRay, v3 *currentPosition);
  void stopThrust();
  void collisionHandler(RayBox *collisionRay, v3 nearObjPos);
  int activeIndex;
  struct objectProperties {
    v3 direction;
    v3 currentPosition;
    v3 previousPosition;
    double length;
  };
  objectProperties dest;
  objectProperties vessel;
  UDPserver *serverConnect;
  int completedRCSOperations;
  double valuesRCS[3];
  double valuesDelta[3];
  std::string operation = "";
  std::string detail = "";
  std::string cl_file = "";
  bool isYaw = false;
  bool isPitch = false;
  bool onCourse = false;
  double countIterations = 0;
  double objSize = 0;
  int debugID;
};

#endif //NAVAP_H
