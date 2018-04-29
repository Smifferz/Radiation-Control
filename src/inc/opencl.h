#ifndef OPENCL_H
#define OPENCL_H

#include <vector>
#include "AOCLUtils/aocl_utils.h"
#include "CL/cl.hpp"

using namespace aocl_utils;


extern cl_platform_id platform;                                                                             
extern unsigned num_devices;                                                                                
extern scoped_array<cl_kernel> kernel;                                                                      
extern scoped_array<cl_command_queue> queue;                                                                
extern std::vector<scoped_array<cl_mem>> inputs;                                                            
extern std::vector<scoped_array<cl_mem>> outputs;                                                           
extern cl_program program;                                                                                  
extern cl_context context;       


class OpenCL
{
 public:
  OpenCL(unsigned num_items);
  bool init_opencl();
  void init_problem();
  void run();
 private:
  cl_platform_id platform;
  scoped_array<cl_device_id> device;
  scoped_array<unsigned> n_per_device;
  unsigned N;
};

#endif
