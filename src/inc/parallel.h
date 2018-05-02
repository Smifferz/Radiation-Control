#ifndef PARALLEL_H
#define PARALLEL_H

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

//#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#include "CL/cl.h"
#include "AOCLUtils/aocl_utils.h"


using namespace aocl_utils;

/* Encapsulates objects that we use the same on most programs.
 * This excludes, notably, buffers. */
typedef struct {
    cl_command_queue command_queue;
    cl_context context;
    cl_device_id device;
    cl_kernel kernel;
    cl_program program;
} Parallel;

void parallel_assert_success(cl_int ret) {
    assert(ret == CL_SUCCESS);
}

char* parallel_read_file(const char* path, long* length_out) {
  //char* buffer;
  //FILE* f;
  //long length;

  //f = fopen(path, "r");
  //assert(NULL != f);
  //fseek(f, 0, SEEK_END);
  //    length = ftell(f);
  //    fseek(f, 0, SEEK_SET);
  //  buffer = new char [length];
  //  if (fread(buffer, 1, length, f) < (size_t)length) {
  //      return NULL;
  //  }
  //  fclose(f);
  //  if (NULL != length_out) {
  //      *length_out = length;
  //  }
  std::cout << "Path: " << path << std::endl;
  std::string buffer;
  std::string filepath(path);
  std::ifstream ifs(filepath);
  if (ifs.is_open()) {
    std::stringstream line;
    line << ifs.rdbuf();
    buffer.append(line.str());
  }
  char* cstr = new char[buffer.length() + 1];
  strcpy(cstr, buffer.c_str());
  *length_out = buffer.length();
  return cstr;
}

char* parallel_read_file_null(const char* path) {
    char* f, *f_tmp;
    long length;
    f_tmp = parallel_read_file(path, &length);
    std::cout << "f_tmp: " << f_tmp << std::endl;
    f = new char[length+1];
    *f = *f_tmp;
    delete []f_tmp;
    f[length] = '\0';
    return f_tmp;
}

void parallel_build_program(
    Parallel* parallel,
    const char* options,
    cl_program* program
) {
    char* err;
    cl_int ret;
    size_t err_len;
    ret = clBuildProgram(*program, 1, &(parallel->device), options, NULL, NULL);
    std::cout << "Build return: " << ret << std::endl;
    if (CL_SUCCESS != ret) {
        clGetProgramBuildInfo(*program, parallel->device, CL_PROGRAM_BUILD_LOG, 0, NULL, &err_len);
        err = new char [err_len];
        clGetProgramBuildInfo(*program, parallel->device, CL_PROGRAM_BUILD_LOG, err_len, err, NULL);
        fprintf(stderr, "error: clBuildProgram:\n%s\n", err);
        free(err);
        exit(EXIT_FAILURE);
    }
}

void parallel_create_program(
    Parallel* parallel,
    const char* source,
    const char* options,
    cl_program* program
) {
    *program = clCreateProgramWithSource(parallel->context, 1, &source, NULL, NULL);
    parallel_build_program(parallel, options, program);
}

void parallel_create_kernel(
    Parallel* parallel,
    const char* source,
    const char* options
) {
    if (NULL != source) {
        parallel_create_program(parallel, source, options, &parallel->program);
        parallel->kernel = clCreateKernel(parallel->program, "kmain", NULL);
        assert(NULL != parallel->kernel);
    } else {
        parallel->kernel = NULL;
        parallel->program = NULL;
    }
}

void parallel_create_kernel_file(
    Parallel* parallel,
    const char* source_path,
    const char* options
) {
    char* source;
    source = parallel_read_file_null(source_path);
    parallel_create_kernel(parallel, source, options);
    free(source);
}

void parallel_init_options(
    Parallel* parallel,
    const char* source,
    const char* options
) {
    cl_platform_id platform;

    clGetPlatformIDs(1, &platform, NULL);
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 1, &(parallel->device), NULL);
    parallel->context = clCreateContext(NULL, 1, &(parallel->device), NULL, NULL, NULL);
    parallel->command_queue = clCreateCommandQueue(parallel->context, parallel->device, 0, NULL);
    parallel_create_kernel(parallel, source, options);
}

void parallel_init(
    Parallel* parallel,
    const char* source
) {
    parallel_init_options(parallel, source, "");
}

void parallel_init_file_options(
    Parallel* parallel,
    const char* source_path,
    const char* options
) {
    char* source;
    source = parallel_read_file_null(source_path);
    printf("Source: %s\n", source);
    parallel_init_options(parallel, source, options);
    free(source);
}

void parallel_init_file(
    Parallel* parallel,
    const char* source_path
) {
    parallel_init_file_options(parallel, source_path, "");
}

void parallel_deinit(
    Parallel* parallel
) {
    clReleaseCommandQueue(parallel->command_queue);
    clReleaseProgram(parallel->program);
    if (NULL != parallel->kernel) {
        clReleaseKernel(parallel->kernel);
    }
    if (NULL != parallel->context) {
        clReleaseContext(parallel->context);
    }
#ifdef CL_1_2
    clReleaseDevice(parallel->device);
#endif
}

double parallel_get_nanos(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec + ts.tv_nsec / 1000000000.0;
}

void parallel_vec_print_i(int *vec, size_t n) {
    size_t i;
    for (i = 0; i < n; ++i) {
        printf("%d\n", vec[i]);
    }
}

void parallel_vec_assert_eq_i(int* vec1, int* vec2, size_t n) {
    if (memcmp(vec1, vec2, n * sizeof(*vec1)) != 0) {
        parallel_vec_print_i(vec1, n);
        puts("");
        parallel_vec_print_i(vec2, n);
        assert(0);
    }
}

#endif
