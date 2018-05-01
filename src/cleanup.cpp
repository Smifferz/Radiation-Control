#include "cleanup.h"

using namespace aocl_utils;

cl_platform_id platform = NULL;                                                                               
unsigned num_devices = 0;                                                                                     
scoped_array<cl_kernel> kernel;                                                                               
scoped_array<cl_command_queue> queue;                                                                         
std::vector<scoped_array<cl_mem>> input_bufs;                                                                     
std::vector<scoped_array<cl_mem>> output_bufs;                                                                    
cl_program program = NULL;                                                                                    
cl_context context = NULL;

void cleanup()
{
	for(unsigned i = 0; i < num_devices; ++i) {
		if (kernel && kernel[i]) {
			clReleaseKernel(kernel[i]);
		}
		if (queue && queue[i]) {
			clReleaseCommandQueue(queue[i]);
		}
#if USE_SVM_API == 0
		unsigned num_inputs = input_bufs.size();
		for (unsigned x = 0; x < num_inputs; ++x) {
			if (input_bufs[x] && input_bufs[x][i]) {
				clReleaseMemObject(input_bufs[x][i]);
			}
		}
		unsigned num_outputs = output_bufs.size();
		for (unsigned x = 0; x < num_outputs; ++x) {
			if (output_bufs[x] && output_bufs[x][i]) {
				clReleaseMemObject(output_bufs[x][i]);
			}
		}
#else
		unsigned num_inputs = input_bufs.size();
		for (unsigned x = 0; x < num_inputs; ++x) {
			if (input_bufs[x][i].get()) {
				input_bufs[x][i].reset();
			}
		}
		unsigned num_outputs = output_bufs.size();
		for (unsigned x = 0; x < num_outputs; ++x) {
			if (output_bufs[x][i]->get()) {
				output_bufs[x][i].reset();
			}
		}
#endif
	}

	if (program) {
		clReleaseProgram(program);
	}
	if (context) {
		clReleaseContext(context);
	}
}
