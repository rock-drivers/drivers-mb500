#include "mb500.hh"
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

    gps::MB500 gps;
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
    {
        gps.stopPeriodicData();
        gps.dumpStatus();
    }
    else if (command == "almanac")
    {
        gps.stopPeriodicData();
        gps.dumpAlmanac();
    }
    else if (command == "satellites")
    {
        gps.stopPeriodicData();
        gps.dumpSatellites();
    }
    else if (command == "edge")
        gps.setCodeCorrelatorMode(gps::MB500::EDGE_CORRELATOR);
    else if (command == "strobe")
        gps.setCodeCorrelatorMode(gps::MB500::STROBE_CORRELATOR);
    else if (command == "fixed")
    {
        gps.setPositionFromCurrent();
        gps.setReceiverDynamics(gps::STATIC);
    }
    else if (command == "static")
    {
        gps.resetStoredPosition();
        gps.setReceiverDynamics(gps::STATIC);
    }
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

