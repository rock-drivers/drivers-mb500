#ifndef GPS_TYPES_H
#define GPS_TYPES_H

#include <gps_base/BaseTypes.hpp>

/**
 * Note: This types have been moved to the gps_base library
 */
namespace gps {
    typedef gps_base::Solution Solution;
    typedef gps_base::Time Time;
    typedef gps_base::Position Position;
    typedef gps_base::Errors Errors;
    typedef gps_base::SolutionQuality SolutionQuality;
    typedef gps_base::Satellite Satellite;
    typedef gps_base::SatelliteInfo SatelliteInfo;
    typedef gps_base::ConstellationInfo ConstellationInfo;
    typedef gps_base::UserDynamics UserDynamics;
}

#endif

