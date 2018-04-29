#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#define __CL_ENABLE_EXCEPTIONS
#include <iostream>
#include <vector>
#include "raybox.h"
#include "parallel.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sstream>

#include <CL/cl.hpp>
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

int RayBox::rayOpenCL(Ray ray1, int debug)
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
  source_path = "/media/fat/Radiation-Control-arm/opencl/rayintersect.cl";
  
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
  //assert(NULL != program);
  if (NULL == program) {
    std::cout << "OpenCL implementation failed. Continuing with software implementation." << std::endl;
    return intersect(ray1);
  }
  parallel_assert_success(binary_status);
  parallel_assert_success(errcode_ret);
  free(binary);
  parallel_build_program(&parallel, NULL, &program);
  kernel = clCreateKernel(program, "kmain", &errcode_ret);
  //assert(NULL != kernel);
  if (NULL == kernel) {
    std::cout << "OpenCL implementation failed. Continuing with software implementation." << std::endl;
    return intersect(ray1);
  }
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


bool RayBox::intersectOpenCL(Ray ray1, int debug)
{
  cl_int err = CL_SUCCESS;
  try {
    // get all platforms (drivers)
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if(platforms.size() == 0) {
      std::cout << "No platforms found. Check OpenCL installation!" << std::endl;
      std::cout << "Starting default detector..." << std::endl;                                                  
      return intersect(ray1);
    }
    cl::Platform default_platform = platforms[0];
    if (debug) {
      std::cout << "Using platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << std::endl;
    }
    
    // get default device of the default platform
    // std::vector<cl::Device> all_devices;
    // default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    // if (all_devices.size() == 0) {
    //   std::cout << "No devices found. Check OpenCL installation!" << std::endl;
    //   std::cout << "Starting default detector..." << std::endl;
    //   return intersect(ray1);
    // }

      // read in the kernel and store its contents in a string
    std::string filename = "/media/fat/Radiation-Control-arm/opencl/rayintersect.cl";
    std::cout << "OpenCL file location: " << filename << std::endl;
    std::ifstream ifs(filename);
    std::string kernel_content;
    if (ifs.is_open()) {
      std::stringstream line;
      line << ifs.rdbuf();
      kernel_content.append(line.str());
    }

    cl_context_properties properties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)(default_platform)(), 0};
    cl::Context context(CL_DEVICE_TYPE_ACCELERATOR, properties);

    std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();

    std::cout << devices[0].getInfo<CL_DEVICE_COMPILER_AVAILABLE>() << std::endl;


    // cl::Device default_device = all_devices[0];
    // if (debug) {
    //   std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>() << std::endl;
    //   std::cout << "Device supports OpenCL version: " << default_device.getInfo<CL_DEVICE_VERSION>() << std::endl;
    // }

    //cl::Context context({default_device});
    cl::Program::Sources source(1, std::make_pair(kernel_content.c_str(), strlen(kernel_content.c_str())));
    cl::Program program_ = cl::Program(context, source);

    // cl::Program::Sources sources;

    //std::stringstream buffer;
    //buffer << ifs.rdbuf();
    //std::string kernel_content = buffer.str();

    std::cout << "Kernel content: " << kernel_content.c_str() << std::endl;

    // sources.push_back({kernel_content.c_str(), kernel_content.length()});

    cl::Program program = cl::Program(context, source);
    //cl::Device device = default_device;
    //program_.build(devices);

    //cl::Kernel kernel(program_, "rayintersect", &err);
    if (program_.build({devices}) != CL_SUCCESS) {
      // get build status
      cl_build_status status = program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(devices[0]);
      std::cout << "Status: " << status << std::endl;
      std::string name = devices[0].getInfo<CL_DEVICE_NAME>();
      std::string buildLog = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]);
      std::cout << "Error building: " << name << std::endl;
      std::cout << "Build log: " << buildLog << std::endl;
      std::cout << "Starting default detector..." << std::endl;                                                  
      return intersect(ray1);
    }

    
    // create buffers on the device
    cl::Buffer buffer_origin_data(context, CL_MEM_READ_WRITE, sizeof(double)*3);
    cl::Buffer buffer_direction(context, CL_MEM_READ_WRITE, sizeof(double)*3);
    cl::Buffer buffer_box_centre(context, CL_MEM_READ_WRITE, sizeof(double)*3);
    cl::Buffer buffer_box_width(context, CL_MEM_READ_WRITE, sizeof(double));
    cl::Buffer buffer_collision_data(context, CL_MEM_READ_WRITE, sizeof(double)*3);
    cl::Buffer buffer_impact(context, CL_MEM_READ_WRITE, sizeof(char));
    cl::Buffer buffer_inside(context, CL_MEM_READ_WRITE, sizeof(char));

    cl::CommandQueue queue(context, devices[0], 0, &err);

    // write values to device
    queue.enqueueWriteBuffer(buffer_origin_data, CL_TRUE, 0, sizeof(double)*3, ray1.origin.data);
    queue.enqueueWriteBuffer(buffer_direction, CL_TRUE, 0, sizeof(double)*3, ray1.direction.data);
    queue.enqueueWriteBuffer(buffer_box_centre, CL_TRUE, 0, sizeof(double)*3, box1.centre.data);
    queue.enqueueWriteBuffer(buffer_box_width, CL_TRUE, 0, sizeof(double), &box1.width);
    

    // run the kernel
    cl::KernelFunctor ray_intersect(cl::Kernel(program, "ray_intersect"), queue, cl::NullRange, cl::NDRange(3), cl::NullRange);
    ray_intersect(buffer_origin_data, buffer_direction, buffer_box_centre, buffer_box_width, buffer_collision_data, buffer_impact, buffer_inside);

    char inside;
    // read results from the device
    queue.enqueueReadBuffer(buffer_collision_data, CL_TRUE, 0, sizeof(double)*3, collisionCoord.data);
    queue.enqueueReadBuffer(buffer_impact, CL_TRUE, 0, sizeof(char), &isCoordFound);
    queue.enqueueReadBuffer(buffer_inside, CL_TRUE, 0, sizeof(char), &inside);

    std::cout << "Result: \n";
    for(int i =0; i < 3; i++) {
      std::cout << collisionCoord.data[i] << " ";
    }
    std::cout << "" << std::endl;
    std::cout << "inside: " << inside << std::endl;
    std::cout << "impact: " << isCoordFound;
  }
  catch (cl::Error err) {
    std::cerr
      << "ERROR: "
      << err.what()
      << "("
      << err.err()
      << ")"
      << std::endl;
  }

  return isCoordFound;
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
