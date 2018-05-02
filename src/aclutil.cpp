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

#ifdef LINUX
#include <stdlib.h>
void* acl_aligned_malloc (size_t size) {
    void *result = NULL;
      posix_memalign (&result, ACL_ALIGNMENT, size);
        return result;
}
void acl_aligned_free (void *ptr) {
    free (ptr);
}

#else // WINDOWS
#include <malloc.h>

void* acl_aligned_malloc (size_t size) {
    return _aligned_malloc (size, ACL_ALIGNMENT);
}
void acl_aligned_free (void *ptr) {
    _aligned_free (ptr);
}

#endif // LINUX


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
