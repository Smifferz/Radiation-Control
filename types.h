#pragma once

typedef union {
	double data[3];               ///< array data interface
	struct { double x, y, z; };   ///< named data interface
} v3;
