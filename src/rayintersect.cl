__kernel void ray_intersect (global const double *origin_data, global const double direction, 
                            global const double *box_centre, global const double *box_width,
                            global double *collision_data, global char *impact, global char *inside)
{
    int id = get_global_id(0);
    char quadrant[3];
    double hit[3];
    double maxT[3]
    double candidatePlane[3];

    if (origin_data[id] < box_centre[id] - (box_width / 2)) {
        quadrant[id] = 1;
        candidatePlane[id] = box_centre[id] - (box_width / 2);
        inside = false;
    } else if (origin_data[id] > box_centre[id] - (box_width / 2)) {
        quadrant[id] = 2;
        candidatePlane[id] = box_centre[id] + (box_width / 2);
        inside = false;
    } else {
        quadrant[id] = 0
    }

    if (inside) {
        collision_data = origin_data;
    }

    if (quadrant[id] != 0 && direction[id] != 0.0) {
        maxT[id] = (candidatePlane[id] - origin_data[id] / direction[id]);
    } else {
        maxT[id] = -1.0;
    }

    int whichPlane = 0
    if (maxT[whichPlane] < maxT[id]) {
        whichPlane = id
    }

    // check final candidate actually inside box
    if (maxT[whichPlane] < 0.0) {
        return false;
    }

    if (whichPlane != id) {
        hit[id] = origin_data[id] + maxT[whichPlane * direction[id];
        if (hit[id] < (box_centre[id] - (box_width / 2)) {
            collision_data = hit;
            impact = false
        }
    } else {
        hit[id] = candidatePlane[id]
    }

    collision_data = hit;
    impact = true;
}



    

