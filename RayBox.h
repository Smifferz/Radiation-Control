#pragma once
#include "types.h"

#define NUMDIM 3	// number of dimensions
#define BOUNDARY_RIGHT	0
#define BOUNDARY_LEFT	1
#define MIDDLE	2

class RayBox
{
private:
	struct Box {
		VECTOR3 centre;
		double width;
		double height;
	} box1;	// Bounding box around target object
	struct AABB {
		VECTOR3 leftBot;	// Minimal coordinates
		VECTOR3 rightTop;	// Maximal corner
	};	// Axis-Aligned Bounding Box
	VECTOR3 collisionCoord;
	bool isCoordFound = false;
public:
	RayBox(VECTOR3 centrePos, double radius);
	~RayBox();
	struct Ray {
		VECTOR3 direction;
		VECTOR3 origin;
	} vessel_ray;
	bool intersect(Ray ray1);
	void findCollisionCoord(Ray ray1, VECTOR3 impactCoord);
	void getCollisionCoord(VECTOR3 impactCoord);

};
