#include <iostream>
#include "compute.h"
#include "cleanup.h"


bool g_enable_notifications = true;       // set to false to temporarily disable printing of error notification callbacks
void oclNotify(const char* errinfo, const void* private_info, size_t cb, void* user_data)
{
    if (g_enable_notifications) {
        std::cout << "  OPENCL Notification Callback: " << errinfo << std::endl;
    }
}

cl_platform_id Compute::platform;
cl_device_id Compute::device;
cl_context Compute::context;
cl_command_queue Compute::queue;
cl_int Compute::status;


Compute::~Compute()
{
    freeResources();
}

void Compute::dumpError(const char* str, cl_int status) 
{
    std::cout << str << " Error code: " << status << std::endl;
}

void Compute::dumpInitError()
{
    std::cout << "Failed to initialize the device. Please check the following" << std::endl;
    std::cout << "\t1. The card is visible to your host operating system" << std::endl;
    std::cout << "\t2. There is a valid OpenCL design currently configured on the card" << std::endl;
    std::cout << "\t3. You've installed all necessary drivers" << std::endl;
    std::cout << "\t4. You've linked the host program with the correct libraries for your specific card" << std::endl;
}

void Compute::freeResources()
{
    if (queue) {
        clReleaseCommandQueue(queue);
    }
    if (context) {
        clReleaseContext(context);
    }
}

// Initializes the OpenCL objects
int Compute::init_opencl()
{
    cl_uint num_platforms;
    cl_uint num_devices;

    // don't buffer stdout
    setbuf(stdout, NULL);

    // get the platform ID
    status = clGetPlatformIDs(1, &platform, &num_platforms);
    if (status != CL_SUCCESS) {
        dumpError("Failed clGetPlatformIDs.", status);
        dumpInitError();
        freeResources();
        return 1;
    }
    if (num_platforms != 1) {
        printf("Found %d platforms", num_platforms);
        dumpInitError();
        freeResources();
        return 1;
    }

    // get the Device ID
    status = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 1, &device, &num_devices);
    if (status != CL_SUCCESS) {
        dumpError("Failed clGetDeviceIDs.", status);
        dumpInitError();
        freeResources();
        return 1;
    }
    if (num_devices != 1) {
        printf("WARNING: Found %d OpenCL devics, using first device.", num_devices);
    }

    // create a context
    context = clCreateContext(0, 1, &device, &oclNotify, NULL, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateContext", status);
        freeResources();
        return 1;
    }

    // create a command queue
    queue = clCreateCommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateCommandQueue", status);
        freeResources();
        return 1;
    }
    return 0;
}

cl_platform_id Compute::getPlatform()
{
    return platform;
}

cl_device_id Compute::getDevice()
{
    return device;
}

cl_context Compute::getContext()
{
    return context;
}

cl_command_queue Compute::getQueue()
{
    return queue;
}