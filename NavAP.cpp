// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.		//
// ---------------------------------------------------- //

// Will run in a seperate thread and send the vessel data
// to a server running elsewhere which will manipulate
// the vessel depending on surrounding environment.

#include "NavAP.h"
#include "opcodes.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <math.h>
#include <cmath>
#include <iostream>

#define PI 3.1415

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

NavAP::NavAP(int debug)
{
  activeIndex = 0;
  serverConnect = new UDPserver("192.168.56.102", debug);
  init();
}

// Initialise the variables of the vessels
// present in the simulation
void NavAP::init()
{
  //simTimeOld = 0;
  //horz_speed_old = 0;
  //vert_speed_old = 0;
  //dSimTime = 0;
  //distOld = 0;
  //headingOld = 0;
  //vert_speed_last_zycl = 0;
  // When initialising a new vessel, add one to the index
  g_Dest[activeIndex].isActive = false;
  g_Dest[activeIndex].isSet = false;
  activeIndex++;
}

bool NavAP::check_ping()
{
  if(serverConnect->check_ping()) {
    std::cout << "Connection can be made..." << std::endl;
    return true;
  }
  return false;
}

// Main loop for the automated navigation system
void NavAP::NavAPMain()
{
  // set the destination for the vessel
  VECTOR3 destinationPos;

  operation = "GET_POS";
  detail = "60";
  serverConnect->test(operation, detail, &destinationPos);
  //serverConnect->perform_transfer(GET_POS, 60, &destinationPos);
  std::cout << "The destination is x = " << destinationPos.data[0] << " y = " << destinationPos.data[1] << " z = " << destinationPos.data[2] << std::endl;

  setNavDestination(&destinationPos);

  detail = "0";
  serverConnect->test(operation, detail, &currentPos);
  //serverConnect->perform_transfer(GET_POS, 0, &currentPos);
  std::cout << "The position is x = " << currentPos.data[0] << " y = " << currentPos.data[1] << " z = " << currentPos.data[2] << std::endl;

  // Set the main thrusters
  operation = "SET_THRUST";
  detail = "1";
  serverConnect->test(operation, detail);
  std::cout << "Main thruster has been set" << std::endl;
  // create a 3d-vector for the near objects
  VECTOR3 nearObjPos;
  // while the vessel isn't at the destination
  while (!((currentPos.x < dest.x + 5) && (currentPos.x > dest.x - 5) &&
          (currentPos.y < dest.y + 5) && (currentPos.y > dest.x - 5) &&
          (currentPos.z < dest.z + 5) && (currentPos.z > dest.z - 5)))
    {
      // count the objects currently in the rendered simulation area
      double num_obj = 0;
      operation = "GET_OBJ_COUNT";
      serverConnect->test(operation, detail, &num_obj);
      //serverConnect->perform_transfer(GET_OBJ_COUNT, 0, &num_obj);
      std::cout << "The number of objects is " << num_obj << std::endl;
      for (int i = 0; i < num_obj; i++)
      {
        double is_sim = 0;
        operation = "IS_VESSEL";
        detail = std::to_string(i);
        serverConnect->test(operation, detail, &is_sim);
        //serverConnect->perform_transfer(IS_VESSEL, i, &is_sim);

        if (is_sim == 1.) return;

        // Find the global position of the vessel and possible collision object
        operation = "GET_POS";
        serverConnect->test(operation, detail, &nearObjPos);
        //serverConnect->perform_transfer(GET_POS, i, &nearObjPos);

        // Find the direction vector by subtracting the previous vector position
        // from the new vector position
        double directionX = currentPos.x - oldPos.x;
        double directionY = currentPos.y - oldPos.y;
        double directionZ = currentPos.z - oldPos.z;

        // Create a RayBox object to determine if a collision is likely
        // This will set up a bounding box around the near object so
        // detections can be calculated.
        double objSize = 0;
        operation = "GET_SIZE";
        serverConnect->test(operation, detail, &objSize);
        //serverConnect->perform_transfer(GET_SIZE, i, &objSize);
        RayBox *collisionCheck = new RayBox(nearObjPos, objSize);

        // Generate a Ray using the global position and the direction vector
        // for the vessel
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
          collisionCheck->findCollisionCoord(collisionCheck->vessel_ray,
                                             collisionCoord);

          // Create the direction vectors between the vessel and collision coord
          VECTOR3 collisionDir;
          collisionDir.x = collisionCoord.x - currentPos.x;
          collisionDir.y = collisionCoord.y - currentPos.y;
          collisionDir.z = collisionCoord.z - currentPos.z;

          // Finding the coordinate for a point on the associated edge of
          // a collision object based on the mean radius can be performed
          // via rearranging the equation for finding the distance between
          // two vector coordinates
          // example:
          // Ex = Cx - sqrt(pow(radius, 2.0)-pow(Cy-Ey, 2.0)-pow(Cz - Ez, 2.0))
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
            if (abs(distFromCentre.data[index]) >
                abs(distFromCentre.data[distIndex]))
              distIndex = index;
          }

          double currentBank, currentYaw, currentPitch, bankset, yawset,
              pitchset = 0;


          // Perform an adjustment to the direction of the vessel
          switch (distIndex)
          {
            case 0:
              // Largest in the x axis, move along horizontal axis
              // Will require change in bank and roll
              operation = "GET_BANK";
              detail = "0";
              serverConnect->test(operation, detail, &currentBank);
              operation = "GET_YAW";
              serverConnect->test(operation, detail, &currentYaw);
              bankset = currentBank / 2;
              std::cout << "\tbankset is " << bankset << std::endl;
              yawset = currentYaw / 2;
              if (bankset > 0.1) bankset = 0.1;
              if (yawset > 0.1) yawset = 0.1;
              setBankSpeed(bankset);
              setYawSpeed(yawset);
              completedRCSOperations = 3;
              break;
            case 1:
              // Largest in the y axis, move along vertical axis
              // Requires change in pitch and maybe roll
              operation = "GET_PITCH";
              serverConnect->test(operation, detail, &currentPitch);
              operation = "GET_YAW";
              serverConnect->test(operation, detail, &currentYaw);
              pitchset = currentPitch / 2;
              yawset = currentYaw / 2;
              // If the set values are larger than the max, set to max
              if (pitchset > 0.1) pitchset = 0.1;
              if (yawset > 0.1) yawset = 0.1;
              setPitchSpeed(pitchset);
              setYawSpeed(yawset);
              completedRCSOperations = 5;
              break;
          }
          // Check if the direction reduces the distance to collision
          // point on the collision object
          bool pathCollision = true;
          while (pathCollision)
          {
            // Create new ray collider object
            RayBox *newRay = new RayBox(nearObjPos, objSize);

            // Setup the ray properties for the collider
            setupNewRay(newRay, &currentPos);

            // Store the 3D collision coordinate
            VECTOR3 newCollide;
            // Store the direction vectors
            VECTOR3 newDirection;

            // Distance to collision
            double prevDistance;
            double nextDistance;
            bool ifNewCollide = newRay->intersect(newRay->vessel_ray);
            // If an intersection takes place, determine the collision
            // coordinates
            if (ifNewCollide)
            {
              // Get collision coordinate
              newRay->findCollisionCoord(newRay->vessel_ray, newCollide);

              // Generate new direction vectors of collision
              newDirection.x = newCollide.x - newRay->vessel_ray.direction.x;
              newDirection.y = newCollide.y - newRay->vessel_ray.direction.y;
              newDirection.z = newCollide.z - newRay->vessel_ray.direction.z;

              // Find the distance to the collision with the
              // new direction vectors
              nextDistance = getDistance(newDirection);
            }
            else
            {
              // The current path no longer results in a collision
              pathCollision = false;
              break;
            }


            // If it does then continue on that path
            // While a collision occurs, keep going in that direction
            while(ifNewCollide) {

              setupNewRay(newRay, &currentPos);

              ifNewCollide = newRay->intersect(newRay->vessel_ray);

              if (!ifNewCollide) {
                break;
              }

              // Get new collision coordinate
              newRay->findCollisionCoord(newRay->vessel_ray, newCollide);

              // Generate the new direction vectors of collision
              newDirection.x = newCollide.x - newRay->vessel_ray.direction.x;
              newDirection.y = newCollide.y - newRay->vessel_ray.direction.y;
              newDirection.z = newCollide.z - newRay->vessel_ray.direction.z;

              // Store the old distance and get a new one
              prevDistance = nextDistance;
              nextDistance = getDistance(newDirection);

              if(nextDistance < prevDistance) {
                // Revert the thrusters to move in the opposite direction
                // Should keep note of which thrusters were changed previously
                // and revert them here
                switch(completedRCSOperations)
                {
                    // TODO: May replace these functions with the setRoll() and
                    // setPitch() functions as they have a better control loop
                  // Bank and Yaw operations have been performed
                  case 3:
                    // Perform the opposite operation to what has been
                    // done previously
                    setBankSpeed((valuesDelta[0] * -1 ));
                    setYawSpeed((valuesDelta[2] * -1));
                    completedRCSOperations = 0;
                    break;
                  case 5:
                    setPitchSpeed((valuesDelta[1] * -1));
                    setYawSpeed((valuesDelta[2] * -1));
                    completedRCSOperations = 0;
                    break;
                  default:
                    std::cout << "RCS operations couldn't be determined"
                              << std::endl;
                    break;
                }
              }
            }
          }

          // Can reset the RCS thrusters to 0 so the vessel moves in a straight
          // line again
          setPitch(0);
          setRoll(0);

        }
        // Ensure the vessel is en-route to the destination

        double relativeAngle = getRelativeHeadingAngle();
        // Determine if the relative angle is within appropriate limits
        while(relativeAngle > 5 || relativeAngle < -5) {
            if (relativeAngle > 180) {
                //TODO: Need to check if there is a getYaw() function
                setYawSpeed(0.04);
            }
            else {
                setYawSpeed(-0.04);
            }
            // Check the current relative angle between the bearings
            relativeAngle = getRelativeHeadingAngle();
        }
      }
    }
}



// Store an input vector into the destination vector
void NavAP::setNavDestination(VECTOR3 *targetDest)
{
  for(int i = 0; i < 3; i++) {
    dest.data[i] = targetDest->data[i];
  }
}

// Get the airspeed angle using oapiGetAirspeedVector
// and return angle
double NavAP::getAirspeedAngle()
{
  VECTOR3 speedVector;
  std::string operation = "GET_AIRSPEED";
  std::string detail = "0";
  serverConnect->test(operation, detail, &speedVector);
  //serverConnect->perform_transfer(GET_AIRSPEED, 0, &speedVector);
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

// Set the bank speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setBankSpeed(double value)
{
  VECTOR3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->test(operation, detail, &currentRotVel);
  double deltaVel = value - currentRotVel.z;
  std::cout << "\tdeltavel is " << deltaVel << std::endl;
  // Reset the RCS thrusters to 0 so a bank maneouver
  // is only attempted in a single direction, then set
  // the thrust in a gtiven direction based of the delta velocity
  operation = "SET_BANK";
  detail = std::to_string(deltaVel);
  serverConnect->test(operation, detail, &valuesRCS[0]);
  valuesDelta[0] = deltaVel;
}

// Set the pitch speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setPitchSpeed(double value)
{
  VECTOR3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->test(operation, detail, &currentRotVel);
  double deltaVel = value - currentRotVel.x;
  // Reset the RCS thrusters to 0 so a pitch maneouver
  // is only attempted in a single direction
  operation = "SET_PITCH";
  detail = std::to_string(deltaVel);
  serverConnect->test(operation, detail, &valuesRCS[1]);
  valuesDelta[1] = deltaVel;
}

// Set the yaw speed using the angular velocity of the vessel
// to set the thrusters in a given direction
void NavAP::setYawSpeed(double value)
{
  VECTOR3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->test(operation, detail, &currentRotVel);
  double deltaVel = value - (-currentRotVel.y);
  // Reset the RCS thrusters to 0 so a yaw maneouver
  // is only attempted in a single direction
  operation = "SET_YAW";
  detail = std::to_string(deltaVel);
  serverConnect->test(operation, detail, &valuesRCS[2]);
  valuesDelta[2] = deltaVel;
}

// Set pitch of vessel relative to previous pitch
void NavAP::setPitch(double pitch)
{
  if (pitch > 1.5) pitch = 1.5;
  if (pitch < -1.5) pitch = -1.5;
  double currentPitch;
  operation = "GET_PITCH";
  detail = "0";
  std::cout << "Sending operation " << operation << " : " << detail << std::endl;
  serverConnect->test(operation, detail, &currentPitch);
  double deltaPitch = currentPitch - pitch;
  double pitchSpeed = deltaPitch * 0.1;
  if (pitchSpeed > 0.04) pitchSpeed = 0.04;
  if (pitchSpeed < -0.04) pitchSpeed = -0.04;
  setPitchSpeed(-pitchSpeed);
}

// Set roll of vessel relative to previous bank
void NavAP::setRoll(double roll)
{
  roll = -roll;
  double currentBank;
  operation = "GET_BANK";
  detail = "0";
  std::cout << "Sending operation " << operation << " : " << detail << std::endl;
  serverConnect->test(operation, detail, &currentBank);
  double deltaBank = currentBank - roll;
  double bankSpeed = deltaBank * 0.1;
  if (bankSpeed > 0.04) bankSpeed = 0.04;
  if (bankSpeed < -0.04) bankSpeed = -0.04;
  setBankSpeed(bankSpeed);
}

// Returns the normalised direction to the set target
void NavAP::setDir(VECTOR3 *dir)
{
  VECTOR3 vesselPos;
  operation = "GET_POS";
  detail = "0";
  serverConnect->test(operation, detail, &vesselPos);
  VECTOR3 targetPos = dest;
  // Find the heading to target destination
  VECTOR3 heading;
  for (int i = 0; i < NUMDIM; i++) {
    heading.data[i] = targetPos.data[i] - vesselPos.data[i];
  }
  double distance = getDistance(heading);
  // Normalise the heading
  VECTOR3 direction;
  for (int i = 0; i < NUMDIM; i++) {
    direction.data[i] = heading.data[i] / distance;
  }
  *dir = direction;
}

// Returns the normalised direction of the current heading
void NavAP::getHeading(VECTOR3 *heading)
{
  // Store previous current position
  VECTOR3 tempPos = currentPos;
  operation = "GET_POS";
  detail = "0";
  serverConnect->test(operation, detail, &currentPos);
  // Find the current heading of vessel
  for(int i = 0; i < NUMDIM; i++) {
    heading->data[i] = currentPos.data[i] - tempPos.data[i];
  }
  double distance = getDistance(*heading);
  // Normalise the heading
  for(int i = 0; i < NUMDIM; i++) {
    heading->data[i] = heading->data[i] / distance;
  }
}

// Get the distance from the vessel to the target position
double NavAP::getDistance(VECTOR3 heading)
{
  float power = 2.0;
  double headingDistX = pow(heading.x, power);
  double headingDistY = pow(heading.y, power);
  double headingDistZ = pow(heading.z, power);
  double headingDistance = sqrt(headingDistX + headingDistY + headingDistZ);
  return headingDistance;
}



void NavAP::setupNewRay(RayBox *ray, VECTOR3 *currentPosition)
{
    // Store the previous position to an old position
    oldPos.x = currentPosition->x;
    oldPos.y = currentPosition->y;
    oldPos.z = currentPosition->z;
    // Get the latest position of the vessel
    operation = "GET_POS";
    detail = "0";
    serverConnect->test(operation, detail, &currentPos);

    // Find direction vectors of new position
    double newXDirection = currentPos.x - oldPos.x;
    double newYDirection = currentPos.y - oldPos.y;
    double newZDirection = currentPos.z - oldPos.z;

    // Use new vectors to check collision again

    // Set the properties of the collision ray
    ray->vessel_ray.origin = currentPos;
    ray->vessel_ray.direction.x = newXDirection;
    ray->vessel_ray.direction.y = newYDirection;
    ray->vessel_ray.direction.z = newZDirection;
}

double NavAP::dot(VECTOR3 headingA, VECTOR3 headingB)
{
  return(headingA.x * headingB.x +
         headingA.y * headingB.y +
         headingA.z * headingB.z);
}

double NavAP::findAngleFromDot(double dot)
{
  return acos(dot);
}

double NavAP::getRelativeHeadingAngle()
{
    // Set the Normalised direction of the vessel
    VECTOR3 direction;
    setDir(&direction);

    // Get the current heading of the vessel
    VECTOR3 currentHeading;
    getHeading(&currentHeading);

    // Find the dot product using the normalised headings
    double dotHeading = dot(direction, currentHeading);

    // Get the angle from the dot product
    double angle = findAngleFromDot(dotHeading);

    return angle;
}
