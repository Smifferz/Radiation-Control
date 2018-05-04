#ifndef COMPUTE_H
#define COMPUTE_H

#include <vector>
#include <functional>
#include "aclutil.h"
#include "CL/cl.hpp"
#include "stdio.h"

//using namespace aocl_utils;
using namespace std::placeholders;

class Compute
{
 public:
    Compute() {}
    ~Compute();
    int init_opencl();
    cl_platform_id getPlatform();
    cl_device_id getDevice();
    cl_context getContext();
    cl_command_queue getQueue();
    template<class T>
    int transferToDevice(T* src, int n, cl_mem* mem);
    template<class T>
    int transferFromDevice(T* dst, int n, cl_mem* mem);
 private:
    static cl_platform_id platform;
    static cl_device_id device;
    static cl_context context;
    static cl_command_queue queue;
    static cl_int status;
    static void dumpError(const char* str, cl_int status);
    static void dumpInitError();
    static void freeResources();
};


template<class T>
int Compute::transferToDevice(T* src, int n, cl_mem* mem)
{
	printf("Transferring buffer to device...\n");
    if (*mem) {
        clReleaseMemObject(*mem);
    }


	printf("Creating input buffer...\n");
    // create the input buffer
    *mem = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(src), NULL, &status);
    if (status != CL_SUCCESS) {
        dumpError("Failed clCreateBuffer for input", status);
        return 1;
    }

	printf("Writing input buffer...\n");
    // write the input
    status = clEnqueueWriteBuffer(queue, *mem, CL_TRUE, 0, sizeof(src), (void*)src, 0, NULL, NULL);
    if (status != CL_SUCCESS) {
        dumpError("Failed to enqueue buffer write", status);
        return 1;
    }
    return 0;
}
template<class T>
int Compute::transferFromDevice(T* dst, int n, cl_mem* mem)
{
    if (n == 0) {
        return 1;
    }
    if (!mem) {
        return 1;
    }
    status = clEnqueueReadBuffer(queue, *mem, CL_TRUE, 0, sizeof(dst), (void*)dst, 0, NULL, NULL);
    if (status != CL_SUCCESS) {
        dumpError("Failed to enqueue buffer read", status);
        return 1;
    }
    return 0;
}
#endif
