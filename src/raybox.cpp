#include "raybox.h"
#include "parallel.h"


#define BIN_PATH __FILE__ ".bin.tmp"

// Fast Ray-Box Intersection

// ==============================================================
//
// RayBox.cpp
//
// Computes the Ray-Box Intersection from a ray (direction vector)
// and a bounding box. Calculates by presuming a ray is being
// emitted from the vessel and determines if that ray would collide
// with the box around the target object.
// ==============================================================

// Creates a box around the target object
RayBox::RayBox(v3 centrePos, double radius)
{
  // Setup geometry for bounding box
  box1.centre = centrePos;
  box1.width = radius * 2;
  box1.height = radius * 2;
}

RayBox::~RayBox()
{
  // reset the coordinate found check to false
  // so subsequent calls
  isCoordFound = false;
}

// Determine if the ray is going to intersect with the bounding box
bool RayBox::intersect(Ray ray1)
{
  v3 hitCoord;
  char inside = true;	// assume the ray starts inside the box
  char quadrant[NUMDIM];
  int i;
  int whichPlane;
  double maxT[NUMDIM];
  double candidatePlane[NUMDIM];

  // Find candidate planes
  for (i = 0; i < NUMDIM; i++)
  {
    // Check how the origin of the ray relates to the coordinates of the bounding box
    if (ray1.origin.data[i] < (box1.centre.data[i] - (box1.width / 2)))
    {
      quadrant[i] = BOUNDARY_LEFT;
      candidatePlane[i] = box1.centre.data[i] - (box1.width / 2);
      inside = false;
    }
    else if (ray1.origin.data[i] > (box1.centre.data[i] + (box1.width / 2)))
    {
      quadrant[i] = BOUNDARY_RIGHT;
      candidatePlane[i] = box1.centre.data[i] + (box1.width / 2);
      inside = false;
    }
    else
      quadrant[i] = MIDDLE;
  }

  // Ray origin inside bounding box
  if (inside)
  {
    collisionCoord = ray1.origin;
    isCoordFound = true;
    return isCoordFound;
  }

  // Calculate T distances to candidate planes
  for (i = 0; i < NUMDIM; i++)
    if (quadrant[i] != MIDDLE && ray1.direction.data[i] != 0.)
      maxT[i] = (candidatePlane[i] - ray1.origin.data[i] / ray1.direction.data[i]);
    else
      maxT[i] = -1.;

  // Get largest of the maxT's for final choice of intersection
  whichPlane = 0;
  for (i = 1; i < NUMDIM; i++)
    if (maxT[whichPlane] < maxT[i])
      whichPlane = i;

  // Check final candidate actually inside box
  if (maxT[whichPlane] < 0.) return false;
  for (i = 0; i < NUMDIM; i++)
  {
    if (whichPlane != i)
    {
      hitCoord.data[i] = ray1.origin.data[i] + maxT[whichPlane] * ray1.direction.data[i];
      if (hitCoord.data[i] < (box1.centre.data[i] - (box1.width / 2)) || hitCoord.data[i] > (box1.centre.data[i] + (box1.width / 2)))
        collisionCoord = hitCoord;
        isCoordFound = false;
        return isCoordFound;
    }
    else
      hitCoord.data[i] = candidatePlane[i];
  }
  collisionCoord = hitCoord;
  isCoordFound = true;
  return isCoordFound;
}

int RayBox::rayOpenCL(Ray ray1)
{
  Parallel parallel;
  FILE* f;
  char* binary, *source_path;
  cl_int input[] = {1, 2}, errcode_ret, binary_status;
  cl_kernel kernel;    
  cl_mem buffer;       
  cl_program program;   
  const size_t global_work_size = sizeof(input) / sizeof(input[0]);
  long length;
  size_t binary_size;
  source_path = "rayintersect.cl";
  
  /* Get the binary and create a kernel with it */
  parallel_init_file(&parallel, source_path);
  clGetProgramInfo(parallel.program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &binary_size, NULL);
  binary = new char [binary_size];
  clGetProgramInfo(parallel.program, CL_PROGRAM_BINARIES, binary_size, &binary, NULL);
  f = fopen(BIN_PATH, "w");
  fwrite(binary, binary_size, 1, f);
  fclose(f);

  program = clCreateProgramWithBinary(
    parallel.context, 1, &parallel.device, &binary_size,
    (const unsigned char **)&binary, &binary_status, &errcode_ret
  );
  assert(NULL != program);
  parallel_assert_success(binary_status);
  parallel_assert_success(errcode_ret);
  free(binary);
  parallel_build_program(&parallel, NULL, &program);
  kernel = clCreateKernel(program, "kmain", &errcode_ret);
  assert(NULL != kernel);
  parallel_assert_success(errcode_ret);

  /* Run the kernel created from the binary */
  buffer = clCreateBuffer(parallel.context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(input), input, NULL);
  clSetKernelArg(kernel, 0, sizeof(buffer), &buffer);
  clEnqueueNDRangeKernel(parallel.command_queue, kernel, 1, NULL, &global_work_size, NULL, 0, NULL, NULL);
  clFlush(parallel.command_queue);
  clFinish(parallel.command_queue);
  clEnqueueReadBuffer(parallel.command_queue, buffer, CL_TRUE, 0, sizeof(input), input, 0, NULL, NULL);

  /* Assertions */
  assert(input[0] == 2);
  assert(input[1] == 3);

  /* Cleanup */
  clReleaseKernel(kernel);
  clReleaseProgram(program);
  clReleaseMemObject(buffer);
  parallel_deinit(&parallel);
  return EXIT_SUCCESS;
}

// Once we know there is a collision on the path, we can calculate
// the predicted collision coordinate and adjust the orientation
// of the vessel to move in the appropriate direction away from
// the obstacle.
void RayBox::findCollisionCoord(Ray ray1, v3 impactCoord)
{
  getCollisionCoord(impactCoord);

  // No longer care about whether the vessel is inside the collision object
  // due to the fact we already know a collision is present
  char quadrant[NUMDIM];
  int i;
  int whichPlane;
  double maxT[NUMDIM];
  double candidatePlane[NUMDIM];

  // Find candidate planes
  for (i = 0; i < NUMDIM; i++)
  {
    // Check how the origin of the ray relates to the coordinates of the bounding box
    if (ray1.origin.data[i] < (box1.centre.data[i] - (box1.width / 2)))
    {
      quadrant[i] = BOUNDARY_LEFT;
      candidatePlane[i] = box1.centre.data[i] - (box1.width / 2);
    }
    else if (ray1.origin.data[i] > (box1.centre.data[i] + (box1.width / 2)))
    {
      quadrant[i] = BOUNDARY_RIGHT;
      candidatePlane[i] = box1.centre.data[i] + (box1.width / 2);
    }
  }

  // Calculate T distances to candidate planes
  for (i = 0; i < NUMDIM; i++)
    if (quadrant[i] != MIDDLE && ray1.direction.data[i] != 0.)
      maxT[i] = (candidatePlane[i] - ray1.origin.data[i] / ray1.direction.data[i]);
    else
      maxT[i] = -1.;

  // Get largest of the maxT's for final choice of intersection
  whichPlane = 0;
  for (i = 1; i < NUMDIM; i++)
    if (maxT[whichPlane] < maxT[i])
      whichPlane = i;

  for (i = 0; i < NUMDIM; i++)
  {
    if (whichPlane != i)
    {
      collisionCoord.data[i] = ray1.origin.data[i] + maxT[whichPlane] * ray1.direction.data[i];
    }
    else
      collisionCoord.data[i] = candidatePlane[i];
  }
  isCoordFound = true;
  impactCoord = collisionCoord;
}

// Sets the impactCoord 3D vector to be equal to the collision
// coordinate if the collision coordinate has been found
void RayBox::getCollisionCoord(v3 impactCoord)
{
  if (!isCoordFound)
  {
    return;
  }

  impactCoord = collisionCoord;
}
