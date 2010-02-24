#include "dgps.hh"
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
using namespace std;

void usage()
{
    cerr << "usage: dgps_tool <device> <cold-reset|warm-reset|status|almanac|moving|satellites|edge|strobe>" << endl;
}

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4)
    {
        usage();
        return 1;
    }

    string device  = argv[1];
    string command = argv[2];

    DGPS gps;
    if (!gps.openSerial(device))
    {
        cerr << "cannot open " << device << endl;
        return 1;
    }

    if (command == "cold-reset")
        gps.reset(true);
    else if (command == "warm-reset")
        gps.reset(false);
    else if (command == "status")
        gps.dumpStatus();
    else if (command == "almanac")
        gps.dumpAlmanac();
    else if (command == "satellites")
        gps.dumpSatellites();
    else if (command == "edge")
        gps.setCodeCorrelatorMode(DGPS::EDGE_CORRELATOR);
    else if (command == "strobe")
        gps.setCodeCorrelatorMode(DGPS::STROBE_CORRELATOR);
    else if (command == "moving")
    {
        gps.resetStoredPosition();
        gps.setReceiverDynamics(gps::ADAPTIVE);
    }
    else if (command == "period")
    {
        if (argc != 4)
        {
            cerr << "missing period argument for 'dgps_tool period'" << endl;
            usage();
            return 1;
        }

        gps.setProcessingRate(boost::lexical_cast<int>(argv[3]));
    }
    else
        usage();

}

