#ifndef COMPUTE_H
#define COMPUTE_H

#include <vector>
#include "AOCLUtils/aocl_utils.h"
#include "CL/cl.hpp"

using namespace aocl_utils;


extern cl_platform_id platform;                                                                             
extern unsigned num_devices;                                                                                
extern scoped_array<cl_kernel> kernel;                                                                      
extern scoped_array<cl_command_queue> queue;                                                                
extern std::vector<scoped_array<cl_mem>> input_bufs;                                                            
extern std::vector<scoped_array<cl_mem>> output_bufs;                                                           
extern cl_program program;                                                                                  
extern cl_context context;       


class Compute
{
 public:
  Compute(unsigned num_items);
  bool init_opencl();
  void init_problem();
  void run();
 private:
  cl_platform_id platform;
  scoped_array<cl_device_id> device;
  scoped_array<unsigned> n_per_device;
  unsigned N;
#if USE_SVM_API == 0
  std::vector<scoped_array<scoped_aligned_ptr<double> > > inputs;
  std::vector<scoped_array<scoped_aligned_ptr<double> > > outputs;
#else
  std::vector<scoped_array<scoped_SVM_aligned_ptr><double> > > inputs;
  std::vector<scoped_array<scoped_SVM_aligned_ptr><double> > > outputs;
#endif
  scoped_array<scoped_array<double> > ref_output;
};

#endif
