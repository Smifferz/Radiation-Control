__kernel void ray_intersect (__global const double *origin_data, __global const double *direction, 
                            __global const double *box_centre, __global const double *box_width,
                            __global double *collision_data, __global char *impact, __global char *inside)
{
    int id = get_global_id(0);
    char quadrant[3];
    double hit[3];
    double maxT[3];
    double candidatePlane[3];
    *inside = true;

    if (origin_data[id] < box_centre[id] - (*box_width / 2)) {
        quadrant[id] = 1;
        candidatePlane[id] = box_centre[id] - (*box_width / 2);
        *inside = false;
    } else if (origin_data[id] > box_centre[id] - (*box_width / 2)) {
        quadrant[id] = 2;
        candidatePlane[id] = box_centre[id] + (*box_width / 2);
        *inside = false;
    } else {
        quadrant[id] = 0;
    }

    if (*inside) {
        collision_data = origin_data;
    }

    if (quadrant[id] != 0 && direction[id] != 0.0) {
        maxT[id] = (candidatePlane[id] - origin_data[id] / direction[id]);
    } else {
        maxT[id] = -1.0;
    }

    int whichPlane = 0;
    if (maxT[whichPlane] < maxT[id]) {
        whichPlane = id;
    }

    if (whichPlane != id) {
        hit[id] = origin_data[id] + maxT[whichPlane] * direction[id];
        if (hit[id] < (box_centre[id] - *box_width / 2)) {
            *collision_data = *hit;
            *impact = false;
        }
    } else {
        hit[id] = candidatePlane[id];
    }

    *collision_data = *hit;
    *impact = true;
}
