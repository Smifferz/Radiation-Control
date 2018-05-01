
// ---------------- Navigation Autopilot -------------- //
// Needs to be portable so as to perform the navigation //
// calculations on FPGA and just send requests for	//
// more information from the simulator.		//
// ---------------------------------------------------- //

// Will run in a seperate thread and send the vessel data
// to a server running elsewhere which will manipulate
// the vessel depending on surrounding environment.

#include "navap.h"
#include "opcodes.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <math.h>
#include <cmath>
#include <iostream>

#define PI 3.1415

NavAP::NavAP(int debug)
{
  serverConnect = new UDPserver("169.254.87.118", debug);
  debugID = debug;
}

/**
 * Initialise the variables of the vessels present in the simulation
 * @brief Initialise vessel variables
 */
void NavAP::init()
{
  for (int i = 0; i < NUMDIM; i++) {
    vessel.currentPosition.data[i] = 0;
    vessel.previousPosition.data[i] = 0;
    vessel.direction.data[i] = 0;
    dest.currentPosition.data[i] = 0;
    dest.previousPosition.data[i] = 0;
    dest.direction.data[i] = 0;
  }
  // set the destination for the vessel
  v3 destinationPos;

  operation = "GET_POS";
  detail = "60";
  serverConnect->transfer_data(operation, detail, &destinationPos);
  setNavDestination(destinationPos);

}

/**
 * Initial ping check between client/server to ensure connection can be made
 * @brief Check for connection
 */
bool NavAP::check_ping()
{
  if(serverConnect->check_ping()) {
    std::cout << "Connection can be made..." << std::endl;
    init();
    return true;
  }
  return false;
}

/**
 * Main loop to perform navigation techniques and call appropriate functions
 * @brief Main navigation loop
 */
void NavAP::NavAPMain()
{
  if (debugID) {
    std::cout << "Running in debug mode" << std::endl;
  } else {
    std::cout << "Running in normal mode" << std::endl;
  }
  // get the position of the vessel
  operation = "GET_POS";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &vessel.currentPosition);

  // Set the main thrusters
  operation = "SET_THRUST";
  detail = "1";
  int thrustCheck;
  serverConnect->transfer_data(operation, detail, &thrustCheck);

  // while the vessel isn't at the destination
  while (!((vessel.currentPosition.x < dest.currentPosition.x + 5) && (vessel.currentPosition.x > dest.currentPosition.x - 5) &&
           (vessel.currentPosition.y < dest.currentPosition.y + 5) && (vessel.currentPosition.y > dest.currentPosition.x - 5) &&
           (vessel.currentPosition.z < dest.currentPosition.z + 5) && (vessel.currentPosition.z > dest.currentPosition.z - 5)))
  {
    // count the objects currently in the rendered simulation area
    int num_obj = 0;
    operation = "GET_OBJ_COUNT";
    serverConnect->transfer_data(operation, detail, &num_obj);

    if (debugID) {
      std::cout << "The number of objects is " << num_obj << std::endl;
    }
    for (int obj_it = 0; obj_it < num_obj; obj_it++)
    {
      int is_sim = 0;
      operation = "IS_VESSEL";
      detail = std::to_string(obj_it);
      serverConnect->transfer_data(operation, detail, &is_sim);

      if (is_sim == 1) {
	return;
      }

      // Find the global position of the vessel and possible collision object
      v3 nearObjPos;
      operation = "GET_POS";
      detail = std::to_string(obj_it);
      serverConnect->transfer_data(operation, detail, &nearObjPos);


      // Get the new current position and store the old
      for(int i = 0; i < NUMDIM; i++) {
        vessel.previousPosition.data[i] = vessel.currentPosition.data[i];
      }
      operation = "GET_POS";
      detail = "0";
      serverConnect->transfer_data(operation,detail, &vessel.currentPosition);
      // Find the direction vector by subtracting the previous vector position
      // from the new vector position
      double directionX = vessel.currentPosition.x - vessel.previousPosition.x;
      double directionY = vessel.currentPosition.y - vessel.previousPosition.y;
      double directionZ = vessel.currentPosition.z - vessel.previousPosition.z;

      // Create a RayBox object to determine if a collision is likely
      // This will set up a bounding box around the near object so
      // detections can be calculated.
      operation = "GET_SIZE";
      detail = std::to_string(obj_it);
      serverConnect->transfer_data(operation, detail, &objSize);

      RayBox *collisionCheck = new RayBox(nearObjPos, objSize);

      // Generate a Ray using the global position and the direction vector
      // for the vessel
      collisionCheck->vessel_ray.origin = vessel.currentPosition;
      collisionCheck->vessel_ray.direction.x = directionX;
      collisionCheck->vessel_ray.direction.y = directionY;
      collisionCheck->vessel_ray.direction.z = directionZ;


      // Check if there is a collision object on the current path
      if (debugID) {
	std::cout << "Checking collision..." << std::endl;
      }
      
      bool ifCollide = collisionCheck->intersect(collisionCheck->vessel_ray);

      isCollision = ifCollide;
      if (ifCollide)
      {
        printf("Collision detected!\n");
        collisionHandler(collisionCheck, nearObjPos);
      }
    }
    // make sure the thrusters are set to zero
    stopThrust();


    //  Get the current position of vessel
    operation = "GET_POS";
    detail = "0";
    serverConnect->transfer_data(operation, detail, &vessel.currentPosition);

    // Declare the variables to hold the vector angles
    double ax, ay, az, ax_dest, ay_dest, az_dest;

    // Calculate in radians the value for the angle for each component
    //ax = atan2(sqrt(pow(vessel.currentPosition.y,2) + pow(vessel.currentPosition.z,2)), vessel.currentPosition.x);
    ax = atan2(vessel.currentPosition.y, vessel.currentPosition.x);
    //ay = atan2(sqrt(pow(vessel.currentPosition.x,2) + pow(vessel.currentPosition.z,2)), vessel.currentPosition.y);
    ay = atan2(vessel.currentPosition.x, vessel.currentPosition.y);
    az = atan2(sqrt(pow(vessel.currentPosition.x,2) + pow(vessel.currentPosition.y,2)), vessel.currentPosition.z);


    printf("Angle for x component of vessel = %lf\n", ax);
    printf("Angle for y component of vessel = %lf\n", ay);
    printf("Angle for z component of vessel = %lf\n", az);

    //ax_dest = atan2(sqrt(pow(dest.currentPosition.x,2) + pow(dest.currentPosition.z,2)), dest.currentPosition.x);
    ax_dest = atan2(dest.currentPosition.y, dest.currentPosition.x);
    //ay_dest = atan2(sqrt(pow(dest.currentPosition.x,2) + pow(dest.currentPosition.z,2)), dest.currentPosition.y);
    ay_dest = atan2(dest.currentPosition.x, dest.currentPosition.y);
    az_dest = atan2(sqrt(pow(dest.currentPosition.x,2) + pow(dest.currentPosition.y,2)), dest.currentPosition.z);

    printf("Angle for x component of dest = %lf\n", ax_dest);
    printf("Angle for y component of dest = %lf\n", ay_dest);
    printf("Angle for z component of dest = %lf\n", az_dest);


    bool thrustSet = false;
    int thrustModifier = 0;
    // Compare the angles of each of the components between vessel and dest
    if (onCourse == false) {
      while(abs(ax - ax_dest) > 0.2) {
        if(ax-ax_dest > 0 && !thrustSet) {
          setYawSpeed(-0.04);
          thrustSet = true;
          thrustModifier = 1;
        }
        else if(ax-ax_dest < 0 && thrustModifier == 1) {
          setYawSpeed(0.04);
          thrustModifier = 2;
          thrustSet = true;
        }
        else if(ax-ax_dest < 0 && !thrustSet) {
          setYawSpeed(0.04);
          thrustSet = true;
          thrustModifier = 2;
        }
        else if(ax-ax_dest > 0 && thrustModifier == 2) {
          setYawSpeed(-0.04);
          thrustModifier = 1;
          thrustSet = true;
        }
        // stop thrusters to continue ascent
        stopThrust();
        // Get the new angle
        //  Get the current position of vessel
        operation = "GET_POS";
        detail = "0";
        serverConnect->transfer_data(operation, detail, &vessel.currentPosition);
        //ax = atan2(sqrt(pow(vessel.currentPosition.y,2) + pow(vessel.currentPosition.z,2)), vessel.currentPosition.x);#
        ax = atan2(vessel.currentPosition.y, vessel.currentPosition.x);
        printf("Angle for x component of vessel = %lf\n", ax);
        printf("Difference between the x-components = %lf\n", ax-ax_dest);
      }
    }
    printf("x component angles are aligned\n");
    stopThrust();
    thrustSet = false;
    thrustModifier = 0;

    if (onCourse == false) {
      while(abs(ay - ay_dest) > 0.2) {
        if(ay-ay_dest > 0 && !thrustSet) {
          setPitchSpeed(-0.04);
          thrustSet = true;
          thrustModifier = 1;
        }
        else if(ay-ay_dest < 0 && thrustModifier == 1) {
          setPitchSpeed(0.04);
          thrustModifier = 2;
          thrustSet = true;
        }
        else if(ay-ay_dest < 0 && !thrustSet) {
          setPitchSpeed(0.04);
          thrustSet = true;
          thrustModifier = 2;
        }
        else if(ay-ay_dest > 0 && thrustModifier == 2) {
          setPitchSpeed(-0.04);
          thrustModifier = 1;
          thrustSet = true;
        }
        // stop thrusters to continue ascent
        stopThrust();
        // Get the new angle
        //  Get the current position of vessel
        operation = "GET_POS";
        detail = "0";
        serverConnect->transfer_data(operation, detail, &vessel.currentPosition);
        //ax = atan2(sqrt(pow(vessel.currentPosition.y,2) + pow(vessel.currentPosition.z,2)), vessel.currentPosition.x);#
        ay = atan2(vessel.currentPosition.x, vessel.currentPosition.y);
        printf("Angle for y component of vessel = %lf\n", ay);
        printf("Difference between the y-components = %lf\n", ay-ay_dest);
      }
    }

    onCourse = true;

    printf("y component angles are aligned\n");
    stopThrust();
    thrustSet = false;
    thrustModifier = 0;
    countIterations++;
    v3 currentRotVel;
    while(abs(ay-ay_dest) < 0.2 && abs(ax-ax_dest) < 0.2) {
      printf("On course, performing minor adjustments\n");
      getCurrentRotVel(&currentRotVel);
      setPitchSpeed(currentRotVel.x);
      usleep(1000 * 20);
      setYawSpeed(-currentRotVel.y);
      usleep(1000*200);
      stopThrust();
      setPitchSpeed(currentRotVel.x);
      usleep(1000*20);
      setYawSpeed(currentRotVel.y);
      usleep(1000*200);
    }
  }
}



/**
 * Store an input coordinate vector into the destination vector
 * @brief Set navigation destination
 * @param targetDest v3 representation of destination coordinates
 */
void NavAP::setNavDestination(v3 targetDest)
{
  for(int i = 0; i < 3; i++) {
    dest.currentPosition.data[i] = targetDest.data[i];
  }
}

/**
 * Get the airspeed angle using oapiGetAirspeedVector
 * @brief Get current airspeed angle
 * @return angle in radians
 */
double NavAP::getAirspeedAngle()
{
  v3 speedVector;
  std::string operation = "GET_AIRSPEED";
  std::string detail = "0";
  serverConnect->transfer_data(operation, detail, &speedVector);
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

/**
 * Get the current rotational velocity of vessel
 * @brief Get current rotational velocity
 * @param *currentRotVel Pointer to the v3 variable to store the result
 */
void NavAP::getCurrentRotVel(v3 *currentRotVel)
{
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->transfer_data(operation, detail, currentRotVel);
}

/**
 * Set the bank speed using the angular velocity of the vessel to set the thrusters in a given direction
 * @brief Set the bank speed
 * @param value Bank velocity
 */
void NavAP::setBankSpeed(double value)
{
  v3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->transfer_data(operation, detail, &currentRotVel);
  double deltaVel = value - currentRotVel.z;
  // Reset the RCS thrusters to 0 so a bank maneouver
  // is only attempted in a single direction, then set
  // the thrust in a gtiven direction based of the delta velocity
  operation = "SET_BANK";
  detail = std::to_string(deltaVel);
  serverConnect->transfer_data(operation, detail, &valuesRCS[0]);
  valuesDelta[0] = deltaVel;
}

/**
 * Set the pitch speed using the angular velocity of the vessel to set the thrusters in a given direction
 * @brief Set the pitch speed
 * @param value Pitch velocity
 */
void NavAP::setPitchSpeed(double value)
{
  v3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->transfer_data(operation, detail, &currentRotVel);
  double deltaVel = value - currentRotVel.x;
  //std::cout << "\tdeltavel : " << deltaVel << std::endl;
  // Reset the RCS thrusters to 0 so a pitch maneouver
  // is only attempted in a single direction
  operation = "SET_PITCH";
  detail = std::to_string(deltaVel);
  serverConnect->transfer_data(operation, detail, &valuesRCS[1]);
  valuesDelta[1] = deltaVel;
}

/**
 *  Set the yaw speed using the angular velocity of the vessel *to set the thrusters in a given direction
 * @brief Set the yaw speed
 * @param value Yaw velocity
 */
void NavAP::setYawSpeed(double value)
{
  v3 currentRotVel;
  std::string operation = "GET_ANG_VEL";
  std::string detail = "0";
  serverConnect->transfer_data(operation, detail, &currentRotVel);
  double deltaVel = value - (-currentRotVel.y);
  //std::cout << "\tdeltavel : " << deltaVel << std::endl;
  // Reset the RCS thrusters to 0 so a yaw maneouver
  // is only attempted in a single direction
  operation = "SET_YAW";
  detail = std::to_string(deltaVel);
  serverConnect->transfer_data(operation, detail, &valuesRCS[2]);
  valuesDelta[2] = deltaVel;
}

/**
 * Set pitch of vessel relative to previous pitch
 * @brief Set pitch
 * @param pitch Pitch to set
 */
void NavAP::setPitch(double pitch)
{
  if (pitch > 1.5) pitch = 1.5;
  if (pitch < -1.5) pitch = -1.5;
  double currentPitch;
  operation = "GET_PITCH";
  detail = "0";
  //std::cout << "Sending operation " << operation << " : " << detail << std::endl;
  serverConnect->transfer_data(operation, detail, &currentPitch);
  // std::cout << "Current pitch : " << currentPitch << std::endl;
  double deltaPitch = currentPitch - pitch;
  double pitchSpeed = deltaPitch * 0.1;
  if (pitchSpeed > 0.04) pitchSpeed = 0.04;
  if (pitchSpeed < -0.04) pitchSpeed = -0.04;
  setPitchSpeed(-pitchSpeed);
}

/**
 * Get the current pitch of vessel
 * @brief Get current pitch
 * @return Current pitch of vessel
 */
double NavAP::getPitch()
{
  double currentPitch;
  operation = "GET_PITCH";
  detail = "0";
  //std::cout << "Sending operation " << operation << " : " << detail << std::endl;
  serverConnect->transfer_data(operation, detail, &currentPitch);
  return currentPitch;
}

/**
 *  Set roll of vessel relative to previous bank
 * @brief Set roll
 * @param roll Roll to set
 */
void NavAP::setRoll(double roll)
{
  roll = -roll;
  double currentBank;
  operation = "GET_BANK";
  detail = "0";
  //std::cout << "Sending operation " << operation << " : " << detail << std::endl;
  serverConnect->transfer_data(operation, detail, &currentBank);
  // std::cout << "Current bank : " << currentBank << std::endl;
  double deltaBank = currentBank - roll;
  double bankSpeed = deltaBank * 0.1;
  if (bankSpeed > 0.04) bankSpeed = 0.04;
  if (bankSpeed < -0.04) bankSpeed = -0.04;
  setBankSpeed(bankSpeed);
}

/**
 * Get the current bank of vessel
 * @brief Get current bank
 * @return Current bank of vessel
 */
double NavAP::getBank()
{
  double currentBank;
  operation = "GET_BANK";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &currentBank);
  return currentBank;
}

/**
 * Get the current yaw of vessel
 * @brief Get current yaw
 * @return Current yaw of vessel
 */
double NavAP::getYaw()
{
  double currentYaw;
  operation = "GET_YAW";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &currentYaw);
  return currentYaw;
}

/**
 * Set yaw of vessel relative to previous yaw
 * @brief Set yaw
 * @param yaw Yaw to set
 */
void NavAP::setYaw(double yaw)
{
  if (yaw > 1.5) yaw = 1.5;
  if (yaw < -1.5) yaw = -1.5;
  double currentYaw;
  operation = "GET_YAW";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &currentYaw);
  //std::cout <<"Current yaw : " << currentYaw << std::endl;
  double deltaYaw = currentYaw - yaw;
  double yawSpeed = deltaYaw * 0.1;
  // std::cout << "yaw speed : " << yawSpeed << std::endl;
  if (yawSpeed > 0.04) yawSpeed = 0.04;
  if (yawSpeed < -0.04) yawSpeed = -0.04;
  setYawSpeed(yawSpeed);
}

/**
 * Set the direction of the vessel
 * @brief Set direction 
 * @param *dir Pointer to direction vector to write to
 * @param normal Specify if normalized direction is required
 */
void NavAP::setDir(v3 *dir, bool normal)
{
  v3 vesselPos;
  operation = "GET_POS";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &vesselPos);
  v3 targetPos = dest.currentPosition;
  // Find the heading to target destination
  v3 heading;
  for (int i = 0; i < NUMDIM; i++) {
    heading.data[i] = targetPos.data[i] - vesselPos.data[i];
  }
  double distance = getDistance(heading);
  // Normalise the heading
  v3 direction;
  if (normal) {
    for (int i = 0; i < NUMDIM; i++) {
      direction.data[i] = heading.data[i] / distance;
    }
  }
  *dir = direction;
}

/**
 * Get the current heading of vessel
 * @brief Get current heading
 * @param *heading Pointer to heading vector to write to
 * @param normal Specify if normalized heading is required
 */
void NavAP::getHeading(v3 *heading, bool normal)
{
  // Store previous current position
  vessel.previousPosition = vessel.currentPosition;
  for (int i = 0; i < NUMDIM; i++) {
    //std::cout << "tempPos[" << i << "] : " << vessel.previousPosition.data[i] << std::endl;
  }
  operation = "GET_POS";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &vessel.currentPosition);
  // Find the current heading of vessel
  for(int i = 0; i < NUMDIM; i++) {
    heading->data[i] = vessel.currentPosition.data[i] - vessel.previousPosition.data[i];
  }
  double distance = getDistance(*heading);
  //std::cout << "Distance : " << distance << std::endl;
  if(normal) {
    // Normalise the heading
    for(int i = 0; i < NUMDIM; i++) {
      heading->data[i] = heading->data[i] / distance;
    }
  }
}

/**
 * Get the distance from the vessel to the target position
 * @brief Get Distance to target
 * @param heading Current heading of vessel
 * @return The distance along the heading to target
 */
double NavAP::getDistance(v3 heading)
{
  float power = 2.0;
  double headingDistX = pow(heading.x, power);
  double headingDistY = pow(heading.y, power);
  double headingDistZ = pow(heading.z, power);
  double headingDistance = sqrt(headingDistX + headingDistY + headingDistZ);
  return headingDistance;
}


/**
 * Perform the setup for a new ray collision calculation
 * @brief Setup a new ray
 * @param *ray Pointer to RayBox object
 * @param *currentPosition Pointer to position vector to write to
 */
void NavAP::setupNewRay(RayBox *ray, v3 *currentPosition)
{
  // Store the previous position to an old position
  vessel.previousPosition.x = currentPosition->x;
  vessel.previousPosition.y = currentPosition->y;
  vessel.previousPosition.z = currentPosition->z;
  // Get the latransfer_data position of the vessel
  operation = "GET_POS";
  detail = "0";
  serverConnect->transfer_data(operation, detail, &vessel.currentPosition);

  // Find direction vectors of new position
  double newXDirection = vessel.currentPosition.x - vessel.previousPosition.x;
  double newYDirection = vessel.currentPosition.y - vessel.previousPosition.y;
  double newZDirection = vessel.currentPosition.z - vessel.previousPosition.z;

  // Use new vectors to check collision again

  // Set the properties of the collision ray
  ray->vessel_ray.origin = vessel.currentPosition;
  ray->vessel_ray.direction.x = newXDirection;
  ray->vessel_ray.direction.y = newYDirection;
  ray->vessel_ray.direction.z = newZDirection;
}

/**
 * Perform the dot product on the input vectors and return the result
 * @brief Perform dot product
 * @param headingA First input heading
 * @param headingB Second input heading
 * @return The dot product of the two headings
 */
double NavAP::dot(v3 headingA, v3 headingB)
{
  return(headingA.x * headingB.x +
         headingA.y * headingB.y +
         headingA.z * headingB.z);
}

/**
 * Find the angle from the dot product of two vectors
 * @brief Find angle from dot product
 * @param dot Output of dot product
 * @return Angle from dot product
 */
double NavAP::findAngleFromDot(double dot)
{
  return acos(dot);
}

/**
 * Get the relative heading between vessel and destination
 * @brief Get relative heading
 * @return Relative heading angle
 */
double NavAP::getRelativeHeadingAngle()
{
  // Set the Normalised direction of the vessel
  v3 direction;
  setDir(&direction, true);

  // Get the current heading of the vessel
  v3 currentHeading;
  getHeading(&currentHeading, true);

  // Find the dot product using the normalised headings
  double dotHeading = dot(direction, currentHeading);

  // Get the angle from the dot product
  double angle = findAngleFromDot(dotHeading);

  return angle;
}

/**
 * Perform normalisation to a vector
 * @brief Normalise a vector
 * @param normalVector Pointer to vector to normalise
 * @param vectorLength Length of input vector
 */
void NavAP::normalise(v3* normalVector, double vectorLength)
{
  for(int i = 0; i < NUMDIM; i++) {
  normalVector->data[i] = normalVector->data[i] / vectorLength;
  }
}

/**
 * Get the component angle between incident vector lengths
 * @brief Get component angle between two lengths
 * @param adjacent Adjacent length
 * @param hypotenuse Longest length
 * @return The component angle to input lengths
 */
double NavAP::getComponentAngle(double adjacent, double hypotenuse)
{
  return acos(adjacent/hypotenuse);
}

/**
 * Stop the thrusters of the vessel
 * @brief Stop thrusters
 */
void NavAP::stopThrust()
{
  operation = "STOP_THRUST";
  detail = "0";
  double thrust;
  serverConnect->transfer_data(operation,detail, &thrust);
}

/**
 * Collision handler to handle possible incoming collisions
 * @brief Determine collisions
 * @param *collisionCheck Pointer to RayBox object
 * @param nearObjPos Position vector of nearby object
 */
void NavAP::collisionHandler(RayBox *collisionCheck, v3 nearObjPos)
{
  // Create 3D vector for position of collision coordinate
  v3 collisionCoord;

  // Get the coordinates of the collision
  collisionCheck->findCollisionCoord(collisionCheck->vessel_ray,
                                     collisionCoord);

  // Create the direction vectors between the vessel and collision coord
  v3 collisionDir;
  collisionDir.x = collisionCoord.x - vessel.currentPosition.x;
  collisionDir.y = collisionCoord.y - vessel.currentPosition.y;
  collisionDir.z = collisionCoord.z - vessel.currentPosition.z;

  std::cout << "Collision found at coordinate : {";
  for(int i =0; i < NUMDIM; i++) {
    std::cout << " " << collisionCoord.data[i] << " ";
  }
  std::cout << "}" << std::endl;

  // Print current vessel position
  std::cout << "Current vessel position : {";
  for(int i =0; i < NUMDIM; i++) {
    std::cout << " " << vessel.currentPosition.data[i] << " ";
  }
  std::cout << "}" << std::endl;

  // Find how far the collision is
  v3 collisionVector;
  for(int i = 0; i < NUMDIM; i++) {
    collisionVector.data[i] = collisionCoord.data[i] - vessel.currentPosition.data[i];
  }
  double collisionDistance = getDistance(collisionVector);
  std::cout  << "Distance to collision is : " << collisionDistance << std::endl;
  if (collisionDistance > 1000000000) {
    return;
  }
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
  v3 distFromCentre;
  distFromCentre.x = nearObjPos.x - vessel.currentPosition.x;
  distFromCentre.y = nearObjPos.y - vessel.currentPosition.y;
  distFromCentre.z = nearObjPos.z - vessel.currentPosition.z;

  int distIndex = 0;
  for (int index = 1; index < NUMDIM; index++)
  {
    if (abs(distFromCentre.data[index]) >
        abs(distFromCentre.data[distIndex]))
      distIndex = index;
  }

  // Perform an adjustment to the direction of the vessel
  switch (distIndex)
  {
    case 0:
      // Largest in the x axis, move along horizontal axis
      // Will require change in bank and roll
      setPitch(0.08);
      completedRCSOperations = 3;
      break;
    case 1:
      // Largest in the y axis, move along vertical axis
      // Requires change in pitch and maybe roll
      setRoll(0.08);
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
    setupNewRay(newRay, &vessel.currentPosition);

    // Store the 3D collision coordinate
    v3 newCollide;
    // Store the direction vectors
    v3 newDirection;

    // Distance to collision
    double prevDistance;
    double nextDistance;
    bool ifNewCollide = newRay->intersect(newRay->vessel_ray);
    // If an intersection takes place, determine the collision
    // coordinates
    if (ifNewCollide)
    {
      for(int i =0 ; i <NUMDIM; i++) {
        std::cout << "Position "<< i << " : " << vessel.currentPosition.data[i] << std::endl;
      }
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
      for(int i =0 ; i <NUMDIM; i++) {
        std::cout << "Position "<< i << " : " << vessel.currentPosition.data[i] << std::endl;
      }

      setupNewRay(newRay, &vessel.currentPosition);

      ifNewCollide = newRay->intersect(newRay->vessel_ray);

      if (!ifNewCollide) {
        std::cout << "collision avoided" << std::endl;
        for(int i =0 ; i <NUMDIM; i++) {
          std::cout << "Position "<< i << " : " << vessel.currentPosition.data[i] << std::endl;
        }
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
      std::cout << "Previous distance was" << prevDistance << std::endl;
      nextDistance = getDistance(newDirection);
      std::cout << "New distance is " << nextDistance << std::endl;

      // Check if the new distance to the collision is less than the
      // previous distance to collision
      if(nextDistance < prevDistance) {
        std::cout << "Gaining proximity to object, reversing direction" << std::endl;
        // Revert the thrusters to move in the opposite direction
        // Should keep note of which thrusters were changed previously
        // and revert them here
        switch(completedRCSOperations)
        {
          // Bank and Yaw operations have been performed
          case 3:
            // Perform the opposite operation to what has been
            // done previously
            //setBankSpeed((valuesDelta[0] * -1 ));
            //setYawSpeed((valuesDelta[2] * -1));
            std::cout << "Setting pitch to invert direction" << std::endl;
            setPitch(-0.08);
            completedRCSOperations = 0;
            break;
          case 5:
            //setPitchSpeed((valuesDelta[1] * -1));
            //setYawSpeed((valuesDelta[2] * -1));
            setRoll(-0.08);
            completedRCSOperations = 0;
            break;
          default:
            std::cout << "RCS operations couldn't be determined"
                      << std::endl;
            break;
        }
      }
      std::cout << "Keeping course" << std::endl;
    }
  }
}
