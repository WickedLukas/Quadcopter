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
    /*class SinCos
    {
    public:
        SinCos() : m_value(0), m_sin_value(0), m_cos_value(1) {}
        SinCos(double v) : m_value(v), m_sin_value(sin(v)), m_cos_value(cos(v)) {}

        double operator=(double v)
        {
            m_value = v;
            m_sin_value = sin(v);
            m_cos_value = cos(v);

            return m_value;
        }

        const double &sin_value{m_sin_value};
        const double &cos_value{m_cos_value};

    private:
        double m_value;
        double m_sin_value;
        double m_cos_value;
    };*/

    //void moveTarget(double distance_rad, const SinCos &bearing_rad);

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