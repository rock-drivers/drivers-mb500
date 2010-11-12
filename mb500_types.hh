#ifndef DGPS_TYPES_H
#define DGPS_TYPES_H

#ifndef __orogen
#include <vector>
#endif

#include <base/time.h>

namespace gps {
    enum MB500_FIRMWARE_OPTIONS
    {
        MB500_UPDATE_RATE        = 1,
        MB500_RTK_ROVER          = 2,
        MB500_RTK_BASE           = 4,
        MB500_PPS_OUTPUT         = 8,
        MB500_EVENT_MARKER       = 16,
        MB500_SBAS_TRACKING      = 32,
        MB500_GLONASS_TRACKING   = 64,
        MB500_RTK_MOVING_BASE    = 128,
        MB500_HEADING            = 256,
        MB500_ADVANCED_MULTIPATH = 512
    };

    enum MB500_DYNAMICS_MODEL
    {
        MB500_STATIC       = 1,
        MB500_QUASI_STATIC = 2,
        MB500_WALKING      = 3,
        MB500_SHIP         = 4,
        MB500_AUTOMOBILE   = 5,
        MB500_AIRCRAFT     = 6,
        MB500_UNLIMITED    = 7,
        MB500_ADAPTIVE     = 8,
        MB500_USER_DEFINED = 9
    };

    enum MB500_AMBIGUITY_THRESHOLD
    {
        MB500_NO_FIX     = 0,
        MB500_FIX_95_0 = 1,
        MB500_FIX_99_0 = 2,
        MB500_FIX_99_9 = 3
    };

    enum MB500_GNSS_MODE
    {
        MB500_GP_L1   = 0,
        MB500_GPGL_L1 = 1,
        MB500_GP_L2   = 2,
        MB500_GP_L2CS = 3,
        MB500_GPGL_L1L2 = 4,
        MB500_GPGL_L1L2CS = 5
    };
}

#endif
