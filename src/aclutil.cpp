// (C) 1992-2014 Altera Corporation. All rights reserved.                         
// Your use of Altera Corporation's design tools, logic functions and other       
// software and tools, and its AMPP partner logic functions, and any output       
// files any of the foregoing (including device programming or simulation         
// files), and any associated documentation or information are expressly subject  
// to the terms and conditions of the Altera Program License Subscription         
// Agreement, Altera MegaCore Function License Agreement, or other applicable     
// license agreement, including, without limitation, that your use is for the     
// sole purpose of programming logic devices manufactured by Altera and sold by   
// Altera or its authorized distributors.  Please refer to the applicable         
// agreement for further details.                                                 
    

#include <stdio.h>

#define ACL_ALIGNMENT 64


#ifdef _WIN32 // WINDOWS
#include <malloc.h>

/**
 * Ensure memory is aligned on an Altera specified boundary
 * @brief Ensure consistent memory alignment
 * @param size Size of variable to allocate
 */
void* acl_aligned_malloc (size_t size) {
    return _aligned_malloc (size, ACL_ALIGNMENT);
}
/**
 * Free memory aligned to an Altera specified boundary
 * @brief Free Alterea specified memory
 * @param ptr Refrence to memory area
 */
void acl_aligned_free (void *ptr) {
    _aligned_free (ptr);
}
#else
#include <stdlib.h>
/**
 * Ensure memory is aligned on an Altera specified boundary
 * @brief Ensure consistent memory alignment
 * @param size Size of variable to allocate
 */
void* acl_aligned_malloc(size_t size) {
    void *result = NULL;
    posix_memalign(&result, ACL_ALIGNMENT, size);
    return result;
}
/**
 * Free memory aligned to an Altera specified boundary
 * @brief Free Alterea specified memory
 * @param ptr Refrence to memory area
 */
void acl_aligned_free(void *ptr) {
    free(ptr);
}

#endif // LINUX

/**
 * Load an AOCX binary file into memory
 * @brief Load AOCX file
 * @param filename C-like string filename representation
 * @param size_ret Reference to size of file
 * @return result Pointer to memory allocated for file
 */
unsigned char* load_file(const char* filename,size_t*size_ret)
{
   FILE* fp;
   size_t w = 0;
   size_t size = 0;
   unsigned char *result = 0;
   fp = fopen(filename,"rb");
   if (fp == NULL ) {
     printf("Error: failed to open aocx file %s\n",filename);
     return NULL;
   }
   // Get source file length
   fseek(fp, 0, SEEK_END);
   size = ftell(fp);
   rewind(fp);
   result = (unsigned char*)malloc(size);
   if ( !result )  return 0;
   if ( !fp ) return 0;
   w=fread(result,1,size,fp);
   fclose(fp);
   *size_ret = w;
   return result;
}
