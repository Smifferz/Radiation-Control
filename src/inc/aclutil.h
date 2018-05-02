/* (C) 1992-2014 Altera Corporation. All rights reserved.                          */
/* Your use of Altera Corporation's design tools, logic functions and other        */
/* software and tools, and its AMPP partner logic functions, and any output        */
/* files any of the foregoing (including device programming or simulation          */
/* files), and any associated documentation or information are expressly subject   */
/* to the terms and conditions of the Altera Program License Subscription          */
/* Agreement, Altera MegaCore Function License Agreement, or other applicable      */
/* license agreement, including, without limitation, that your use is for the      */
/* sole purpose of programming logic devices manufactured by Altera and sold by    */
/* Altera or its authorized distributors.  Please refer to the applicable          */
/* agreement for further details.                                                  */
    

#ifndef ACLUTIL_H
#define ACLUTIL_H

// Allocate and free memory aligned to value that's good for
// Altera OpenCL performance.
void *acl_aligned_malloc (size_t size);
void  acl_aligned_free (void *ptr);
unsigned char* load_file(const char* filename,size_t*size_ret);

#endif
