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
#include <chrono>
#include "aclutil.h"
#include "compute.h"

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


// ACL runtime configuration
static cl_platform_id platform;
static cl_device_id device;
static cl_context context;
static cl_command_queue queue;
static cl_kernel kernel;
static cl_program program;
static cl_int status;

static cl_mem kernel_dum;
static std::vector<cl_mem> kernel_inputs;
static cl_mem output_1;
static cl_mem output_2;
static cl_mem output_3;

int dum_size;

/** 
 * Free the OpenCL resources if they are instantiated
 * @brief Free any OpenCL resources
 */
static void freeResources()
{
    if (kernel) {
        clReleaseKernel(kernel);
    }
    if (program) {
        clReleaseProgram(program);
    }
    if (queue) {
        clReleaseCommandQueue(queue);
    }
    if (context) {
        clReleaseContext(context);
    }
    if (kernel_dum) {
        clReleaseMemObject(kernel_dum);
    }
    for (int i = 0; i < kernel_inputs.size(); i++) {
        if (kernel_inputs[i]) {
            clReleaseMemObject(kernel_inputs[i]);
        }
    }
	if (output_1) {
		clReleaseMemObject(output_1);
	}
	if (output_2) {
		clReleaseMemObject(output_2);
	}
	if (output_3) {
		clReleaseMemObject(output_3);
	}
}

/**
 * Perform a dump of any OpenCL error code
 * @brief Dump OpenCL errors
 * @param *str C-like char pointeer containing error
 * @param status OpenCL error code
 */
static void dumpError(const char* str, cl_int status)
{
    printf("%s\n", str);
    printf("Error code: %d\n", status);
    freeResources();
}

/**
 * Create a box around the target object related to its centre position and radius
 * @briefCreate a box around target object
 * @param centrePos v3 representation of centre position of object
 * @param radius radius of target object
 */
RayBox::RayBox(v3 centrePos, double radius)
{
  // Setup geometry for bounding box
  box1.centre = centrePos;
  box1.width = radius * 2;
  box1.height = radius * 2;
}

/**
 * Destructor for RayBox class
 * @brief Destructor for RayBox class
 */
RayBox::~RayBox()
{
  // reset the coordinate found check to false
  // so subsequent calls are set false
  isCoordFound = false;
}
/**
 * Determine if the ray is going to intersect with the bounding box on its path
 * @brief Check for ray collision
 * @param ray1 Ray struct
 * @return isCoordFound Boolean result of intersection determination 
 */
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

/**
 * OpenCL implementation of intersect using AOCL binaries
 * @brief AOCL binary OpenCL
 * @param ray Ray struct 
 * @return isCoordFound Boolean value of intersection determination
 */
bool RayBox::clRun(Ray ray)
{
    int n = 3;
//#ifdef ALTERA_CL
  //  globalSize = workSize = (n < 256) ? n : (n / 256) * 256;
//#else
 //   workSize = (n < 256) ? n : 256;
 //   globalSize = (n / 256) * 256;
//#endif
	size_t globalSize[3] = {1, 1, 1};
    Compute* cl_init = new Compute();
    int init = cl_init->init_opencl();
    if (init) {
        std::cout << "WARNING: Could not initialize OpenCL. Running default detector." << std::endl;
        return intersect(ray);
    }
    platform = cl_init->getPlatform();
    device = cl_init->getDevice();
    context = cl_init->getContext();
    queue = cl_init->getQueue();
    cl_int kernel_status;
    cl_int status;
    const char* kernel_name = "ray_intersect";

    // create the kernel
    unsigned char* aocx; size_t aocx_len = 0;
    aocx = load_file("ray_intersect.aocx", &aocx_len);
    if (aocx == NULL) {
        printf("ERROR: Failed to find ray_intersect.aocx...Running default collision detector\n");
        intersect(ray);
    }

    // create the program
    cl_program program = clCreateProgramWithBinary(context, 1, &device, &aocx_len, (const unsigned char**)&aocx, &kernel_status, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateProgramWithBinary.", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    // build the program
    status = clBuildProgram(program, 0, NULL, "", NULL, NULL);
    if (status != CL_SUCCESS) {
        dumpError("Failed clBuildProgram", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    free(aocx);

    // create the kernel
    kernel = clCreateKernel(program, kernel_name, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateKernel", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    printf("Created Kernel %s ...\n", kernel_name);

    // setup inputs to device
	cl_mem tmpInput;
    int transferStatus = cl_init->transferToDevice(&ray.origin.data, n, &tmpInput);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    kernel_inputs.push_back(tmpInput);
    transferStatus = cl_init->transferToDevice(&ray.direction.data, n, &tmpInput);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    kernel_inputs.push_back(tmpInput);
	clReleaseMemObject(tmpInput);
    transferStatus = cl_init->transferToDevice(&box1.centre.data, n, &tmpInput);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    kernel_inputs.push_back(tmpInput);
	clReleaseMemObject(tmpInput);
    transferStatus = cl_init->transferToDevice(&box1.width, 1, &tmpInput);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    kernel_inputs.push_back(tmpInput);

    output_1 = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(double) * n, NULL, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateBuffer for output.", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    output_2 = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(double), NULL, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateBuffer for output.", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    output_3 = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(double), NULL, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateBuffer for output.", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    // setup kernel arguments
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&kernel_inputs[0]);
    status |= clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&kernel_inputs[1]);
    status |= clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&kernel_inputs[2]);
    status |= clSetKernelArg(kernel, 3, sizeof(cl_mem), (void*)&kernel_inputs[3]);
    status |= clSetKernelArg(kernel, 4, sizeof(cl_mem), (void*)&output_1);
    status |= clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&output_2);
    status |= clSetKernelArg(kernel, 6, sizeof(cl_mem), (void*)&output_3);
    if (status != CL_SUCCESS) {
        dumpError("Failed Set args", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    cl_event evt = NULL;
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // launch kernel
    status = clEnqueueNDRangeKernel(queue, kernel, n, NULL, globalSize, NULL, 0, NULL, &evt);
    if (status != CL_SUCCESS) {
        dumpError("Failed to launch kernel", status);
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    // wait for kernel to complete
    clFinish(queue);
	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();                              
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	std::cout << "Time taken to perform OpenCL compute: " << duration << " nanoseconds" << std::endl;	
    char inside;
    // read the results
    transferStatus = cl_init->transferFromDevice(&collisionCoord.data, n, &output_1);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    transferStatus = cl_init->transferFromDevice(&isCoordFound, 1, &output_2);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }
    transferStatus = cl_init->transferFromDevice(&inside, n, &output_3);
    if (transferStatus) {
        printf("Running default collision detector\n");
        return intersect(ray);
    }

    if (inside) {
        collisionCoord = ray.origin;
        isCoordFound = true;
        return isCoordFound;
    }

    // free resources
    freeResources();

    return isCoordFound;
}

/**
 * OpenCL implementation of intersection using the custom OpenCL interface wrapper
 * @brief Custom OpenCL interface wrapper implementation of intersect
 * @param ray1 Ray struct 
 * @param debug Debug mode specifier
 * @return EXIT_SUCCESS Returns success if complete
 */
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

/**
 * OpenCL implementation of intersect using OpenCL 2.2 standards
 * @brief OpenCL 2.2 standard intersect
 * @param ray1 Ray struct
 * @param debug Debug mode specifier
 * @return isCoordFound Boolean value for intersection determination
 */
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
    // The commented out code is OpenCL 1.1
    //cl::KernelFunctor ray_intersect(cl::Kernel(program, "ray_intersect"), queue, cl::NullRange, cl::NDRange(3), cl::NullRange);
    //ray_intersect(buffer_origin_data, buffer_direction, buffer_box_centre, buffer_box_width, buffer_collision_data, buffer_impact, buffer_inside);

    /*auto ray_intersect = cl::make_kernel<cl::Buffer&, cl::Buffer&, cl::Buffer&, cl::Buffer&, cl::Buffer&, cl::Buffer&, cl::Buffer&>(program, "ray_intersect");
    cl::EnqueueArgs eargs(queue, cl::NullRange, cl::NDRange(3), cl::NullRange);
    ray_intersect(eargs, buffer_origin_data, buffer_direction, buffer_box_centre, buffer_box_width, buffer_collision_data, buffer_impact, buffer_inside);
*/
    cl::Kernel kernel_intersect = cl::Kernel(program, "ray_intersect");
    kernel_intersect.setArg(0, buffer_origin_data);
    kernel_intersect.setArg(1, buffer_direction);
    kernel_intersect.setArg(2, buffer_box_centre);
    kernel_intersect.setArg(3, buffer_box_width);
    kernel_intersect.setArg(4, buffer_collision_data);
    kernel_intersect.setArg(5, buffer_impact);
    kernel_intersect.setArg(6, buffer_inside);
    queue.enqueueNDRangeKernel(kernel_intersect, cl::NullRange, cl::NDRange(3), cl::NullRange);
    queue.finish();

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

/**
 * Find the exact collision coordinate of the ray
 * @brief Finds collision coordinate
 * @param ray1 Ray struct
 * @param impactCoord v3 to contain the collision coordinate
 */
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

/**
 * Sets the passed 3D vector to be equal to the collision
 * coordinate if the collision coordinate has been found
 * @brief Gets the collision coordinate
 * @param impactCoord v3 to store the result in
 */
void RayBox::getCollisionCoord(v3 impactCoord)
{
  if (!isCoordFound)
  {
    return;
  }

  impactCoord = collisionCoord;
}
