#include <iostream>
#include "opencl.h"
#include "cleanup.h"

OpenCL::OpenCL(unsigned num_items)
{
  N = num_items;
}

// Initializes the OpenCL objects
bool OpenCL::init_opencl()
{
  cl_int status;

  std::cout << "Initializing OpenCL\n" << std::endl;

  if (!setCwdToExeDir()) {
    return false;
  }

  // Get the OpenCL platform.
  platform = findPlatform("Intel(R) FPGA SDK for OpenCL(TM)");
  if(platform == NULL) {
    printf("ERROR: Unable to find Intel(R) FPGA OpenCL platform.\n");
    return false;
  }
  
  // Query the available OpenCL device
  device.reset(getDevices(platform, CL_DEVICE_TYPE_ALL, &num_devices));
  std::cout << "Platform:" << getPlatformName(platform).c_str() << std::endl;
  std::cout << "Using " << num_devices << "device(s)" << std::endl;
  for(unsigned i = 0; i < num_devices; ++i) {
    std::cout << " " << getDeviceName(device[i]).c_str() << std::endl;
  }

  // Create the context
  context = clCreateContext(NULL, num_devices, device, &oclContextCallback, NULL, &status);
  checkError(status, "Failed to create context");

  // Create a program for all device(s). Use the first device as the type
  // representative device (assuming all device(s) are of the same type).
  std::string binary_file = getBoardBinaryFile("rayintersect", device[0]);
  std::cout << "Using AOCX: " << binary_file.c_str() << std::endl;
  program = createProgramFromBinary(context, binary_file.c_str(), device, num_devices);

  // Build the program that was just created
  status = clBuildProgram(program, 0, NULL, "", NULL, NULL);
  checkError(status, "Failed to build program");

  // Create per-device objects
  queue.reset(num_devices);
  kernel.reset(num_devices);
  n_per_device.reset(num_devices);

  for(unsigned i = 0; i < num_devices; ++i) {
    // Command queue
    queue[i] = clCreateCommandQueue(context, device[i], CL_QUEUE_PROFILING_ENABLE, &status);
    checkError(status, "Failed to create command queue");

    // Kernel
    const char* kernel_name = "rayintersect";
    kernel[i] = clCreateKernel(program, kernel_name, &status);
    checkError(status, "Failed to create kernel");

    // Determine the number of elements processed by this device
    n_per_device[i] = N / num_devices; // number of elements handled by this device

    // Spead out the remainder of the elements over the first
    // N % num_devices
    if (i < (N % num_devices)) {
      n_per_device[i]++;
    }

#if USE_SVM_API == 0
    // Input buffers
    // get the number of input buffers
    unsigned num_inputs = input_bufs.size();
    for(unsigned x = 0; x < num_inputs; x++) {
      input_bufs[x][i] = clCreateBuffer(context, CL_MEM_READ_ONLY,
				    n_per_device[i] * sizeof(float), NULL, &status);
      std::string error_msg = "Failed to create buffer for input" + std::to_string(i);
      checkError(status, error_msg.c_str());
    }

    // Output buffers
    // Get the number of output buffers
    unsigned num_outputs = output_bufs.size();
    for(unsigned x = 0; x < num_outputs; x++) {
      output_bufs[x][i] = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
				      n_per_device[i] * sizeof(float), NULL, &status);
      std::string error_msg = "Failed to create buffer for output" + std::to_string(i);
      checkError(status, error_msg.c_str());
    }
#else
    cl_device_svm_capabilities caps = 0;

    status = clGetDeviceInfo(
			     device[i],
			     CL_DEVICE_SVM_CAPABILITIES,
			     sizeof(cl_device_svm_capabilities),
			     &caps,
			     0
			     );
    checkError(status, "Failed to get device info");

    if (!(caps & CL_DEVICE_SVM_COARSE_GRAIN_BUFFER)) {
      std::cout << "The host was compiled with USE_SVM_API, however the device currently being targeted does not support SVM." << std::endl;
      // Free the resources allocated
      cleanup();
      return false;
    }
#endif  /* USE_SVM_API == 0 */
  }

  return true;
}


// Initialize the data for the problem. Requires num_devices to be known
void OpenCL::init_problem()
{
  if(num_devices == 0) {
    checkError(-1, "No devices");
  }
  
  // get number of inputs
  unsigned num_inputs = inputs.size();
  for (unsigned i = 0; i < num_inputs; i++) {
	  inputs[i].reset(num_devices);
  }
 // get number of outputs
 unsigned num_outputs = outputs.size();
 for (unsigned i = 0; i < num_outputs; i++) {
	 outputs[i].reset(num_devices);
 }

 // Generate inputs vectors and the reference output consisting of 
 // a total of N elements.
 // We create seperate arrays for each device dso that each device 
 // has an aligned buffer.
 for (unsigned i = 0; i < num_devices; ++i) {
#if USE_SVM_API == 0
	for(unsigned x = 0; x < num_inputs; x++) {
		inputs[x][i].reset(n_per_device[i]);
	}
	for (unsigned x = 0; x < num_outputs; x++) {
		outputs[x][i].reset(n_per_device[i]);
	}

	// Need to set up the values here
#endif
 }
}

void OpenCL::run()
{

}
