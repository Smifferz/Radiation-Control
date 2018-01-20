// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.		//
// ---------------------------------------------------- //

// Will run in a seperate thread and send the vessel data
// to a server running elsewhere which will manipulate
// the vessel depending on surrounding environment.

#include "NavAP.h"
#include "RayBox.h"
#include "types.h"
#include "UDPserver.h"
#include <math.h>
#include <cmath>

struct Dest
{
  bool isSet;
  bool isActive;
  double longitude;
  double latitude;
  double dir;
  double dist;
  double dSim;
  double timeToDest;
  char str_lon[255];
  char str_lat[255];
  char str_dist[255];
  char str_dir[255];
  char str_timeToDest[255];
};

Dest g_Dest[256] = {};

NavAP::NavAP()
{
  //TODO: Make a request for the handle of the vessel to undergo
  // navigation processing
  // --UPDATE: Handle won't be sent over network, NavAP will
  // merely tell the simulator what to do with the handle to
  // give it data that it can use
  activeIndex = 0;
  serverConnect = new UDPserver();
  // Make the request for the vessel handle

  // Initialise the vessel
  init(hVessel);
}

// Initialise the variables of the vessels
// present in the simulation
// TODO: Probably not needed anymore as the
// vessel handle version is more consistent
// The reqeust for the OBJHANDLE of the vessel can
// be made over TCP and sent to the server
void NavAP::init(int vesselIndex)
{
  simTimeOld = 0;
  horz_speed_old = 0;
  vert_speed_old = 0;
  dSimTime = 0;
  distOld = 0;
  headingOld = 0;
  vert_speed_last_zycl = 0;
  g_Dest[vesselIndex].isActive = false;
  g_Dest[vesselIndex].isSet = false;
  g_Dest[vesselIndex].hObject = NULL;

  OBJHANDLE hVessel = oapiGetVesselByIndex(vesselIndex);
  activeIndex = vesselIndex;
  if (hVessel != NULL)
    {
      VESSEL *vessel = oapiGetVesselInterface(hVessel);
      for (unsigned int i = 0; i < vessel->GetThrusterCount(); i++)
        vessel->SetThrusterLevel(vessel->GetThrusterHandleByIndex(i), 0);
    }
}
// Initialise the variables of the vessels
// present in the simulation
void NavAP::init(OBJHANDLE hVessel)
{
  simTimeOld = 0;
  horz_speed_old = 0;
  vert_speed_old = 0;
  dSimTime = 0;
  distOld = 0;
  headingOld = 0;
  vert_speed_last_zycl = 0;
  // When initilising a new vessel, add one to the index
  g_Dest[activeIndex].isActive = false;
  g_Dest[activeIndex].isSet = false;
  g_Dest[activeIndex].hObject = NULL;
  activeIndex++;

  //  objhandle hvessel = oapigetvesselbyindex(vesselindex);
  //activeindex = vesselindex;
  if (hVessel != NULL)
    {
      VESSEL *vessel = oapiGetVesselInterface(hVessel);
      for (unsigned int i = 0; i < vessel->GetThrusterCount(); i++)
        vessel->SetThrusterLevel(vessel->GetThrusterHandleByIndex(i), 0);
    }
  //output_redirect = std::thread(&NavAP::NavAPMain, std::ref(hVessel));
}

// Main loop for the automated navigation system
void NavAP::NavAPMain(OBJHANDLE vesselHandle)
{
  // set the destination for the vessel
  //vector3 navdest = setnavdestination();
  OutputHandler();
  VECTOR3 destinationPos;

  // For testing, get find the coordinates of jupiter and use as destination
  OBJHANDLE destObject = oapiGetObjectByName("JUPITER");
  oapiGetGlobalPos(destObject, &destinationPos);

  dest.x = destinationPos.x;
  dest.y = destinationPos.y;
  dest.z = destinationPos.z;

  //objhandle vesselhandle = oapigetvesselbyindex(vesselindex);
  // get the initial position of the vessel
  oapiGetGlobalPos(vesselHandle, &currentPos);
  // create a 3d-vector for the near objects
  VECTOR3 nearObjPos;
  // while the vessel isn't at the destination
  while ((currentPos.x < dest.x + 5) && (currentPos.x > dest.x - 5) &&
         (currentPos.y < dest.y + 5) && (currentPos.y > dest.x - 5) &&
         (currentPos.z < dest.z + 5) && (currentPos.z > dest.z - 5))
    {
      // count the objects currently in the rendered simulation area
      for (int i = 0; i < oapiGetObjectCount(); i++)
        {
          // find the handle for the nearby object
          OBJHANDLE simObj = oapiGetObjectByIndex(i);

          // Check if the nearby object is the vessel
          if (simObj == vesselHandle)
            {
              continue;	// Skip this iteration
            }

          // Find the global position of the vessel and possible collision object
          oapiGetGlobalPos(simObj, &nearObjPos);

          // Find the direction vector by subtracting the previous vector position
          // from the new vector position
          double directionX = currentPos.x - oldPos.x;
          double directionY = currentPos.y - oldPos.y;
          double directionZ = currentPos.z - oldPos.z;

          // Create a RayBox object to determine if a collision is likely
          // This will set up a bounding box around the near object so
          // detections can be calculated.

          RayBox *collisionCheck = new RayBox(nearObjPos, oapiGetSize(simObj));

          // Generate a Ray using the global position and the direction vector for the vessel
          collisionCheck->vessel_ray.origin = currentPos;
          collisionCheck->vessel_ray.direction.x = directionX;
          collisionCheck->vessel_ray.direction.y = directionY;
          collisionCheck->vessel_ray.direction.z = directionZ;


          // Check if there is a collision object on the current path
          bool ifCollide = collisionCheck->intersect(collisionCheck->vessel_ray);

          isCollision = ifCollide;
          if (ifCollide)
            {
              // Create 3D vector for position of collision coordinate
              VECTOR3 collisionCoord;

              // Get the coordinates of the collision
              collisionCheck->findCollisionCoord(collisionCheck->vessel_ray, collisionCoord);

              // Create the direction vectors between the vessel and collision coord
              VECTOR3 collisionDir;
              collisionDir.x = collisionCoord.x - currentPos.x;
              collisionDir.y = collisionCoord.y - currentPos.y;
              collisionDir.z = collisionCoord.z - currentPos.z;

              // Calculate the distance to the collision coordinate
              // NOTE: This is not currently the precise distance if the
              // object is non-cuboid as it treats the incident "face" as
              // a plane so the distance will be constant if square on with
              // the collision object
              double collisionDistance = getDistance(collisionDir);

              // Finding the coordinate for a point on the associated edge of
              // a collision object based on the mean radius can be performed
              // via rearranging the equation for finding the distance between
              // two vector coordinates
              // example:
              // Ex = Cx - sqrt(pow(radius, 2.0) - pow(Cy-Ey, 2.0) - pow(Cz - Ez, 2.0))
              // Where E = edge, C = centre and the x,y and z can be interchanged
              // depending which axis we are investigating




              // Need to determine how the adjustments are to be made
              // Work out which side of the centre of the object we are at
              // and which edge boundary we are closest to escape from
              VECTOR3 distFromCentre;
              distFromCentre.x = nearObjPos.x - currentPos.x;
              distFromCentre.y = nearObjPos.y - currentPos.y;
              distFromCentre.z = nearObjPos.z - currentPos.z;

              int distIndex = 0;
              for (int index = 1; index < NUMDIM; index++)
                {
                  if (abs(distFromCentre.data[index]) > abs(distFromCentre.data[distIndex]))
                    distIndex = index;
                }

              double currentBank, currentYaw, currentPitch, bankset, yawset, pitchset;
              // Perform an adjustment to the direction of the vessel
              switch (distIndex)
                {
                case 0:
                  // Largest in the x axis, move along horizontal axis
                  // Will require change in bank and roll
                  currentBank = vesselAuto->GetBank();
                  currentYaw = vesselAuto->GetYaw();
                  bankset = currentBank / 2;
                  yawset = currentYaw / 2;
                  if (bankset > 0.1) bankset = 0.1;
                  if (yawset > 0.1) yawset = 0.1;
                  setBankSpeed(bankset);
                  setYawSpeed(yawset);
                  break;
                case 1:
                  // Largest in the y axis, move along vertical axis
                  // Requires change in pitch and maybe roll
                  currentPitch = vesselAuto->GetPitch();
                  currentYaw = vesselAuto->GetYaw();
                  pitchset = currentPitch / 2;
                  yawset = currentYaw / 2;
                  // If the set values are larger than the max, set to max
                  if (pitchset > 0.1) pitchset = 0.1;
                  if (yawset > 0.1) yawset = 0.1;
                  setPitchSpeed(pitchset);
                  setYawSpeed(yawset);
                  break;
                  // Looking back, I no longer think there is a need to translate for the Z-axis
                  // due to only seeing a "face" of the object at a time there is no third
                  // dimension for direction adjustment		30/11/2017
                case 2:
                  // Largest in the z axis, move along horizontal axis
                  // Requires change in pitch
                  break;
                }
              // Check if the direction reduces the distance to collision
              // point on the collision object
              bool pathCollision = true;
              while (pathCollision)
                {
                  // Get the latest position of the vessel
                  VECTOR3* newPosition;
                  oapiGetGlobalPos(vesselHandle, newPosition);
                  // Find direction vectors of new position
                  double newXDirection = newPosition->x - currentPos.x;
                  double newYDirection = newPosition->y - currentPos.y;
                  double newZDirection = newPosition->z - currentPos.z;

                  // Use new vectors to check collision again
                  RayBox *newCheck = new RayBox(nearObjPos, oapiGetSize(simObj));

                  // Set the properties of the collision ray
                  newCheck->vessel_ray.origin = *newPosition;
                  newCheck->vessel_ray.direction.x = newXDirection;
                  newCheck->vessel_ray.direction.y = newYDirection;
                  newCheck->vessel_ray.direction.z = newZDirection;

                  bool ifNewCollide = newCheck->intersect(newCheck->vessel_ray);
                  // If an intersection takes place, determine the collision coordinates
                  if (ifNewCollide)
                    {
                      // Store the 3D collision coordinate
                      VECTOR3 newCollide;
                      // Get collision coordinate
                      newCheck->findCollisionCoord(newCheck->vessel_ray, newCollide);

                      // Generate new direction vectors of collision
                      VECTOR3 newDirection;
                      newDirection.x = newCollide.x - newXDirection;
                      newDirection.y = newCollide.y - newYDirection;
                      newDirection.z = newCollide.z - newZDirection;

                      // Find the distance to the collision with the
                      // new direction vectors
                      double newDistance = getDistance(newDirection);
                    }
                  else
                    {
                      // The current path no longer results in a collision
                      pathCollision = false;
                      break;
                    }


                  // If it does then continue on that path

                  // If not then move in opposite direction
                }
            }
        }

      // ---------------------------------------------------------------------------------------//



      // Set main thrusters to progress along path
    }

}


// Reads an input file specifying parameters for
// the target destination for the vessel
VECTOR3 NavAP::setNavDestination()
{
  // Read in input file
  // Parse input file for given destination
  // If no destination given then error
  VECTOR3 targetDest;
  // Populate targetDest with data from file
  //
  return targetDest;
}

// Get vessel acceleration vectors and store into
// their associated variables
//void NavAP::getAccelerations(double dSimTime)
//{
//	double simTimeNew = oapiGetSimTime();
//	VESSEL *V = _VESSEL;
//	VECTOR3 v3;
//	V->GetHorizonAirspeedVector(v3);
//	double horz_speed_new = sqrt((v3.x*v3.x) + (v3.z*v3.z));
//	double vert_speed_new = v3.y;
//	deltaVector = getAirspeedAngle() - getDir();
//	deltaVectorByTime = (deltaVector - deltaVector_old) / dSimTime;
//	horzAcc = (horz_speed_new - horz_speed_old) / dSimTime;
//	verAcc = (vert_speed_new - vert_speed_old) / dSimTime;
//	simTimeOld = simTimeNew;
//	horz_speed_old = horz_speed_new;
//	vert_speed_old = vert_speed_new;
//	deltaVector_old = deltaVector;
//}

// Get the airspeed angle using oapiGetAirspeedVector
// and return angle
double NavAP::getAirspeedAngle()
{
  VECTOR3 speedVector;
  VESSEL *v = _VESSEL;
  oapiGetAirspeedVector(_HVESSEL, &speedVector);
  double angle;
  angle = atan(speedVector.x / speedVector.z);
  double x = speedVector.x;
  double y = speedVector.y;
  if (x > 0 && y > 0) return angle;	// 180 + angle
  if (x > 0 && y < 0) return PI + angle;	// angle
  if (x < 0 && y < 0) return PI + angle;	// 360 + angle
  if (x < 0 && y < 0) return (2 * PI) + angle;	// 360 - angle
  return -1;
}

// Get the relative heading between the vessel and it's destination
//double NavAP::getRelativeAngle()
//{
//	double baseAngle = getDir();
//	double heading;
//	VECTOR3 dir;
//	oapiGetAirspeedVector(_HVESSEL, &dir);
//	OBJHANDLE vH = _HVESSEL;
//	oapiGetHeading(vH, &heading);
//	double diffHeading = baseAngle - heading;
//	if (diffHeading > PI) diffHeading = (-2 * PI) + diffHeading;
//	if (diffHeading < -PI) diffHeading = (2 * PI) + diffHeading;
//	return diffHeading;
//}

// Set the bank speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setBankSpeed(double value)
{
  VECTOR3 currentRotVel;
  vesselAuto->GetAngularVel(currentRotVel);
  double deltaVel = value - currentRotVel.z;
  double thrust = getRCSThrustByDelta(deltaVel);
  // Reset the RCS thrusters to 0 so a bank maneouver
  // is only attempted in a single direction
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
  // Set the thrust in a given direction based on the value of the delta velocity
  if (deltaVel > 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, thrust);
  if (deltaVel < 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, -thrust);
}

// Set the pitch speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setPitchSpeed(double value)
{
  VECTOR3 currentRotVel;
  vesselAuto->GetAngularVel(currentRotVel);
  double deltaVel = value - currentRotVel.x;
  double thrust = getRCSThrustByDelta(deltaVel);
  // Reset the RCS thrusters to 0 so a pitch maneouver
  // is only attempted in a single direction
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
  // Set the thrust in a given direction based on the value of the delta velocity
  if (deltaVel > 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, thrust);
  if (deltaVel < 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, -thrust);
}

// Set the yaw speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setYawSpeed(double value)
{
  VECTOR3 currentRotVel;
  vesselAuto->GetAngularVel(currentRotVel);
  double deltaVel = value - (-currentRotVel.y);
  double thrust = getRCSThrustByDelta(deltaVel);
  // Reset the RCS thrusters to 0 so a yaw maneouver
  // is only attempted in a single direction
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
  vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
  // Set the thrust in a given direction based on the value of the delta velocity
  if (deltaVel > 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, thrust);
  if (deltaVel < 0) vesselAuto->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, -thrust);
}

// Get the RCS thruster level based on the deltaspeed
double NavAP::getRCSThrustByDelta(double deltaSpeed)
{
  double deltaFAC = 20;
  double thAdd = 0.01;
  double thrust = (deltaFAC * deltaSpeed) + thAdd;
  if (thrust > 0. && thrust <= 1.0) return thrust;
  if (thrust > 0. && thrust > 1.0) return 1.0;
  if (thrust < 0. && thrust >= -1.0) return thrust;
  if (thrust < 0. && thrust < -1.0) return -1.0;
  return 0;
}

// Set pitch of vessel relative to previous pitch
double NavAP::setPitch(double pitch)
{
  if (pitch > 1.5) pitch = 1.5;
  if (pitch < -1.5) pitch = -1.5;
  double currentPitch = vesselAuto->GetPitch();
  double deltaPitch = currentPitch - pitch;
  double pitchSpeed = deltaPitch * 0.1;
  if (pitchSpeed > 0.04) pitchSpeed = 0.04;
  if (pitchSpeed < -0.04) pitchSpeed = -0.04;
  setPitchSpeed(-pitchSpeed);
  return 0;
}

// Set roll of vessel relative to previous bank
double NavAP::setRoll(double roll)
{
  roll = -roll;
  //TODO: Add request for bank here
  // Maybe the requests should be done with
  // integer values where the integer corresponds
  // to a function request
  // (These could be stored in a seperate file on
  // the host system)
  double currentBank = vesselAuto->GetBank();
  double deltaBank = currentBank - roll;
  double bankSpeed = deltaBank * 0.1;
  if (bankSpeed > 0.04) bankSpeed = 0.04;
  if (bankSpeed < -0.04) bankSpeed = -0.04;
  setBankSpeed(bankSpeed);
  return 0;
}

// TODO: Need to determine the approach to set
// the direction
double NavAP::setDir(double dir)
{
  return 0;
}

// Returns the normalised direction to the set target
//TODO: This should be completely done on the host
// only the navigation should be done on the FPGA
void NavAP::getDir(VECTOR3 dir)
{
  VECTOR3 vesselPos;
  oapiGetGlobalPos(_HVESSEL, &vesselPos);
  VECTOR3 targetPos = dest;
  VECTOR3 heading = targetPos - vesselPos;
  double distance = getDistance(heading);
  VECTOR3 direction = heading / distance;
  dir = direction;
}

// Get the distance from the vessel to the target position
//TODO: This should be completely done on the host
// only the navigation should be done on the FPGA
double NavAP::getDistance(VECTOR3 heading)
{
  float power = 2.0;
  double headingDistX = pow(heading.x, power);
  double headingDistY = pow(heading.y, power);
  double headingDistZ = pow(heading.z, power);
  double headingDistance = sqrt(headingDistX + headingDistY + headingDistZ);
  return headingDistance;
}
