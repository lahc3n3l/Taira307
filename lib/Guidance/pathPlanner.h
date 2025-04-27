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
    attitude att;
    bool isAttValid; // true if attitude is valid

    float groundSpeed; // in m/s
    bool isGroundSpeedValid; // true if ground speed is valid
    //TODO: add airspeed, vertical speed, etc.
};

class pathPlanner {

    public:

        // default constructor
        pathPlanner();

        // destructor
        ~pathPlanner();

        // function to add a waypoint to the list of waypoints
        void addWaypoint(float lon, float lat, float altitude, float speed);

        void setMission(std::vector<waypoint> waypoints);

        waypoint getNextWaypoint();

        void updateCurrentState(fullState state);


    private:
        std::vector<waypoint> waypoints; // list of waypoints
        fullState currentState; // current state of the vehicle
        int currentWaypointIdx = 0; // index to track the current waypoint
        float arrivalThd = 1.0 ; // threshold to consider the waypoint as reached (in meters)

        
};
#endif // PATH_PLANNER_H
