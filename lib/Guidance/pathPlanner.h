#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <vector>


struct waypoint {
    float lon; // in degrees
    float lat; // in degrees
    float altitude; // in meters
    float speed; // in m/s
};

struct attitude {
    float roll; // in radians
    float pitch; // in radians
    float heading; // in radians
};

struct position {
    float lon; // in degrees
    float lat; // in degrees
    float altitude; // in meters
};

struct fullState {
    position pos;
    attitude att;
    float groundSpeed; // in m/s
    //TODO: add airspeed, vertical speed, etc.
};

#endif // PATH_PLANNER_H
