#include "dgps.hh"
#include <iostream>
#include <string>
using namespace std;

void usage()
{
    cerr << "usage: dgps_tool <device> <cold-reset|warm-reset|status|almanac|moving>" << endl;
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }

    string device  = argv[1];
    string command = argv[2];

    DGPS gps;
    if (!gps.open(device))
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
    else if (command == "moving")
    {
        gps.resetStoredPosition();
        gps.setReceiverDynamics(DGPS::ADAPTIVE);
    }
    else
        usage();

}

