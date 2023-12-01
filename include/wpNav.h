#ifndef WPNAV_H_
#define WPNAV_H_

#include <Location.h>

#include <queue>

enum class WpStatus
{
    finished,
    updating,
    error
};

class WpNav
{
public:
    void init(const NeoGPS::Location_t &currentLocation, const NeoGPS::Location_t &stopLocation, float velocity, float interval_s = 0.001);

    void addWaypoint(const NeoGPS::Location_t &waypoint);

    WpStatus updateTarget(float dt_s, NeoGPS::Location_t &targetLocation);

private:
    const double m_earthRadius_m{NeoGPS::Location_t::EARTH_RADIUS_KM * 1000};
    const double m_minDistance_rad{1 / m_earthRadius_m};

    bool m_initialized{false};
    bool m_bearingCalculated{false};
    
    float m_velocity;
    float m_interval_s;

    float m_sum_dt_s;
    double m_sum_distance_rad;

    NeoGPS::Location_t m_targetLocation{};

    NeoGPS::Location_t m_currentWaypoint{};
    std::queue<NeoGPS::Location_t> m_waypoints;
};

#endif