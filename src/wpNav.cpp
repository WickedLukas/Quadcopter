#include "wpNav.h"
#include "vector"

void WpNav::init(const NeoGPS::Location_t &currentLocation, const NeoGPS::Location_t &stopLocation, float velocity, float interval_s)
{
    m_targetLocation = currentLocation;
    m_currentWaypoint = stopLocation;
    m_velocity = velocity;
    m_interval_s = interval_s;

    m_sum_dt_s = interval_s;
    m_sum_distance_rad = 0;

    m_waypoints = std::queue<NeoGPS::Location_t>{};

    m_bearingCalculated = false;
    m_initialized = true;
}

void WpNav::addWaypoint(const NeoGPS::Location_t &waypoint)
{
    if (!m_initialized)
    {
        return;
    }

    m_waypoints.push(waypoint);
}

WpStatus WpNav::updateTarget(float dt_s, NeoGPS::Location_t &targetLocation)
{
    if (!m_initialized)
    {
        return WpStatus::error;
    }

    static double distance_rad;
    distance_rad = (double) m_velocity * dt_s / m_earthRadius_m;

    m_sum_dt_s += dt_s;
    m_sum_distance_rad += distance_rad;

    static float bearing_rad{0};
    if (!m_bearingCalculated)
    {
        bearing_rad = m_targetLocation.BearingTo(m_currentWaypoint);

        m_bearingCalculated = true;
    }

    static WpStatus wpStatus{WpStatus::updating};
    if (m_sum_dt_s < m_interval_s)
    {
        targetLocation = m_targetLocation;
        return wpStatus;
    }

    if (m_targetLocation.EquirectDistanceRadians(m_currentWaypoint) < max(m_minDistance_rad, m_sum_distance_rad))
    {
        if (m_waypoints.empty())
        {
            targetLocation = m_targetLocation;
            wpStatus = WpStatus::finished;
            return wpStatus;
        }
        else
        {
            m_currentWaypoint = m_waypoints.front();
            m_waypoints.pop();
            m_bearingCalculated = false;
        }
    }

    m_sum_dt_s = 0;
    m_sum_distance_rad = 0;

    m_targetLocation.OffsetBy(m_sum_distance_rad, bearing_rad);
    
    targetLocation = m_targetLocation;
    wpStatus = WpStatus::updating;
    return wpStatus;
}