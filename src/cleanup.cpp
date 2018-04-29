#include "cleanup.h"

using namespace aocl_utils;

cl_platform_id platform = NULL;                                                                               
unsigned num_devices = 0;                                                                                     
scoped_array<cl_kernel> kernel;                                                                               
scoped_array<cl_command_queue> queue;                                                                         
std::vector<scoped_array<cl_mem>> inputs;                                                                     
std::vector<scoped_array<cl_mem>> outputs;                                                                    
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
		unsigned num_inputs = inputs.size();
		for (unsigned x = 0; x < num_inputs; ++x) {
			if (inputs[x] && inputs[x][i]) {
				clReleaseMemObject(inputs[x][i]);
			}
		}
		unsigned num_outputs = outputs.size();
		for (unsigned x = 0; x < num_outputs; ++x) {
			if (outputs[x] && outputs[x][i]) {
				clReleaseMemObject(outputs[x][i]);
			}
		}
#else
		unsigned num_inputs = inputs.size();
		for (unsigned x = 0; x < num_inputs; ++x) {
			if (inputs[x][i].get()) {
				inputs[x][i].reset();
			}
		}
		unsigned num_outputs = outputs.size();
		for (unsigned x = 0; x < num_outputs; ++x) {
			if (outputs[x][i]->get()) {
				outputs[x][i].reset();
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
