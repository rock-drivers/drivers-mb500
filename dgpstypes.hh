#ifndef DGPS_TYPES_H
#define DGPS_TYPES_H

#ifndef __orogen
#include <vector>
#endif

#include <dfki/base_types.h>

namespace gps {
    enum GPS_SOLUTION_TYPES
    {
        NO_SOLUTION  = 0,
        AUTONOMOUS   = 1,
        DIFFERENTIAL = 2,
        UNUSED       = 3,
        RTK_FIXED    = 4,
        RTK_FLOAT    = 5
    };

    enum FIRMWARE_OPTIONS
    {
        UPDATE_RATE        = 1,
        RTK_ROVER          = 2,
        RTK_BASE           = 4,
        PPS_OUTPUT         = 8,
        EVENT_MARKER       = 16,
        SBAS_TRACKING      = 32,
        GLONASS_TRACKING   = 64,
        RTK_MOVING_BASE    = 128,
        HEADING            = 256,
        ADVANCED_MULTIPATH = 512
    };

    struct Solution {
        DFKI::Time timestamp;
        double latitude;
        double longitude;
        GPS_SOLUTION_TYPES positionType;
        int noOfSatellites;
        double altitude;
        double geoidalSeparation;
        double ageOfDifferentialCorrections;

        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
    };

    struct Position {
        DFKI::Time timestamp;
        double latitude;
        double longitude;
        GPS_SOLUTION_TYPES positionType;
        int noOfSatellites;
        double altitude;
        double geoidalSeparation;
        double ageOfDifferentialCorrections;
    };

    struct Errors {
        DFKI::Time timestamp;
        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
    };

    enum CONSTELLATIONS {
        CONSTELLATION_GPS,
        CONSTELLATION_SBAS,
        CONSTELLATION_GLONASS
    };
    struct Satellite {
        int PRN;
        int elevation;
        int azimuth;
        double SNR;

#ifndef __orogen
        CONSTELLATIONS getConstellation() const
        {
            if (PRN < 33)
                return CONSTELLATION_GPS;
            else if (PRN < 65)
                return CONSTELLATION_SBAS;
            else
                return CONSTELLATION_GLONASS;
        }
#endif
    };

    typedef std::vector < gps::Satellite> SatelliteInfo;
}

#endif
