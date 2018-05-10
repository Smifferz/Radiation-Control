#ifndef RAYBOX_H
#define RAYBOX_H

#include "types.h"
#include <CL/cl.hpp>

#define NUMDIM 3	// number of dimensions
#define BOUNDARY_RIGHT	0
#define BOUNDARY_LEFT	1
#define MIDDLE	2

/**                                                                                                           
 * The RayBox class performs collision detection using Ray Box Intersection techniques                                    
 * @brief The class that performs collision detection                                                                  
 */
class RayBox
{
private:
	struct Box {
		v3 centre;
		double width;
		double height;
	} box1;	// Bounding box around target object
	struct AABB {
		v3 leftBot;	// Minimal coordinates
		v3 rightTop;	// Maximal corner
	};	// Axis-Aligned Bounding Box
	v3 collisionCoord;
	bool isCoordFound = false;
public:
	RayBox(v3 centrePos, double radius);
	~RayBox();
	struct Ray {
		v3 direction;
		v3 origin;
	} vessel_ray;
	bool intersect(Ray ray1);
	bool intersectOpenCL(Ray ray1, int debug);
	int rayOpenCL(Ray ray1, int debug);
    bool clRun(Ray ray);
	void findCollisionCoord(Ray ray1, v3 impactCoord);
	void getCollisionCoord(v3 impactCoord);

};

#endif //RAYBOX_H
