#ifndef COMPUTE_H
#define COMPUTE_H

#include <vector>
#include <functional>
#include "aclutil.h"
#include "CL/cl.hpp"

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

#endif
