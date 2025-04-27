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
    bool isPosValid; // true if position is valid
    bool isAttValid; // true if attitude is valid

    bool isAttValid; // true if attitude is valid
    attitude att;
    float groundSpeed; // in m/s
    bool isGroundSpeedValid; // true if ground speed is valid
    //TODO: add airspeed, vertical speed, etc.
};

#endif // PATH_PLANNER_H
