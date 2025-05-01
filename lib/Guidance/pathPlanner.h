#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <vector>


struct waypoint {
    double lon; // in degrees
    double lat; // in degrees
    float altitude; // in meters
    float speed; // in m/s

    void set(double lon, double lat, float altitude, float speed) {
        this->lon = lon;
        this->lat = lat;
        this->altitude = altitude;
        this->speed = speed;
    }
};

struct attitude {
    float roll; // in radians
    float pitch; // in radians
    float heading; // in radians
};

struct position {
    double lon; // in degrees
    double lat; // in degrees
    float altitude; // in meters
};

struct fullState {
    position pos;
    bool isPosValid; // true if position is valid
    attitude att;
    bool isAttValid; // true if attitude is valid

    float groundSpeed; // in m/s
    bool isGroundSpeedValid; // true if ground speed is valid
    //TODO: add airspeed, vertical speed, etc.
};

class pathPlanner {

    public:

        // default constructor
        pathPlanner();;

        // function to add a waypoint to the list of waypoints
        void addWaypoint(double lon, double lat, float altitude, float speed);

        void addWaypoint(waypoint wp) ;
        
        waypoint getNextWaypoint();

        void updateCurrentState(fullState state);

        size_t getNumberOfWaypoints() const;

        void setMission(std::vector<waypoint> waypoints);

    private:
    // the prefix _m is used to indicate that the variable is a member of the class

        std::vector<waypoint> m_waypoints; // list of waypoints
        fullState m_currentState; // current state of the vehicle
        int m_currentWaypointIdx = 0; // index to track the current waypoint
        float m_arrivalThd = 1.0 ; // threshold to consider the waypoint as reached (in meters)

        
};
#endif // PATH_PLANNER_H
