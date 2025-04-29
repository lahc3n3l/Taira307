#include "pathPlanner.h"
#include <iostream>

pathPlanner::pathPlanner()
{
}

void pathPlanner::addWaypoint(float lon, float lat, float altitude, float speed)
{
    waypoint wp;
    wp.set(lon, lat, altitude, speed);
    m_waypoints.push_back(wp);
}

void pathPlanner::addWaypoint(waypoint wp)
{
    m_waypoints.push_back(wp);
}

size_t pathPlanner::getNumberOfWaypoints() const
{
    return m_waypoints.size();
}

void pathPlanner::setMission(std::vector<waypoint> waypoints)
{
    this->m_waypoints = waypoints;
}

waypoint pathPlanner::getNextWaypoint()
{
    if (m_waypoints.empty()) {
        std::cerr << "No waypoints available." << std::endl;
        return waypoint(); // Return an empty waypoint
    }
    
    return m_waypoints.front(); // Return the first waypoint in the list
}