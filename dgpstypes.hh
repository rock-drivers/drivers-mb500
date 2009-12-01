#ifndef DGPS_TYPES_H
#define DGPS_TYPES_H

#ifndef __orogen
#include <vector>
#endif

#include <dfki/time.h>

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
#ifndef __orogen
	Solution()
	    : positionType(INVALID) {}
#endif
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
#ifndef __orogen
	Position()
	    : positionType(INVALID) {}
#endif
    };

    struct Errors {
        DFKI::Time timestamp;
        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
    };

    struct SolutionQuality {
        DFKI::Time timestamp;
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
        DFKI::Time timestamp;
        std::vector < gps::Satellite> knownSatellites;
    };
}

#endif
