#ifndef DGPS_TYPES_H
#define DGPS_TYPES_H

#ifndef __orogen
#include <vector>
#endif

#include <base/time.h>

namespace gps {
    enum GPS_SOLUTION_TYPES
    {
        NO_SOLUTION  = 0,
        AUTONOMOUS   = 1,
        DIFFERENTIAL = 2,
        INVALID      = 3,
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

    enum DYNAMICS_MODEL
    {
        STATIC       = 1,
        QUASI_STATIC = 2,
        WALKING      = 3,
        SHIP         = 4,
        AUTOMOBILE   = 5,
        AIRCRAFT     = 6,
        UNLIMITED    = 7,
        ADAPTIVE     = 8,
        USER_DEFINED = 9
    };

    enum GNSS_MODE
    {
        GP_L1   = 0,
        GPGL_L1 = 1,
        GP_L2   = 2,
        GP_L2CS = 3,
        GPGL_L1L2 = 4,
        GPGL_L1L2CS = 5
    };

    struct Solution {
        base::Time time;
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
#ifndef __orogen
	Solution()
	    : positionType(INVALID) {}
#endif
    };

    struct Position {
        base::Time time;
        double latitude;
        double longitude;
        GPS_SOLUTION_TYPES positionType;
        int noOfSatellites;
        double altitude;
        double geoidalSeparation;
        double ageOfDifferentialCorrections;
#ifndef __orogen
	Position()
	    : positionType(INVALID) {}
#endif
    };

    struct Errors {
        base::Time time;
        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
    };

    struct SolutionQuality {
        base::Time time;
        std::vector<int> usedSatellites;
        double pdop;
        double hdop;
        double vdop;
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
	static CONSTELLATIONS getConstellationFromPRN(int prn)
	{
            if (prn < 33)
                return CONSTELLATION_GPS;
            else if (prn < 65)
                return CONSTELLATION_SBAS;
            else
                return CONSTELLATION_GLONASS;
	}

        CONSTELLATIONS getConstellation() const
        {
	    return getConstellationFromPRN(PRN);
        }
#endif
    };

    struct SatelliteInfo {
        base::Time time;
        std::vector < gps::Satellite> knownSatellites;
    };
}

#endif
