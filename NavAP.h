#pragma once


// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for		//
// more information from the simulator.					//
// ---------------------------------------------------- //

#include "Orbitersdk.h"
#include "OutputHandler.h"
#include "UDPclient.h"
#include "types.h"
#include <thread>

class NavAP
{
public:
	NavAP();
	void init();
	VECTOR3 setNavDestination();
	void NavAPMain();
	void getActiveIndex(int vesselIndex);
    bool isCollision;
	double currentThrust;
    double progressThrust;
private:
	void setBankSpeed(double value);
	void setPitchSpeed(double value);
	void setYawSpeed(double value);
	double setPitch(double pitch);
	double setRoll(double roll);
	double setDir(double dir);
	void getDir(VECTOR3 dir);
	double headingOld;
	double distOld;
	double dSimTime;
	double simTimeOld;
	double deltaVector;
	double deltaVector_old;
	double deltaVectorByTime;
	double horz_speed_old;
	double vert_speed_old;
	double vert_speed_last_zycl;
	double horzAcc;
	double verAcc;
	double HSSum[20][2];
	double getDistance(VECTOR3 heading);
	double getRCSThrustByDelta(double deltaSpeed);
	double getAirspeedAngle();
	int activeIndex;
	VESSEL* vesselAuto;
	VECTOR3 dest;
	VECTOR3 currentPos;
	VECTOR3 oldPos;
	void ThreadHandler();
	bool isThreadOpen = false;
    std::thread output_redirect;
	UDPclient *serverConnect;
};
