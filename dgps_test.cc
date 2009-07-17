#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>

using namespace std;

int main (int argc, const char** argv){
    DGPS gps;

    if (argc != 3)
    {
        cerr << "usage: dgps_test device_name port_name" << endl;
        return 1;
    }

    string device_name = argv[1];
    string port_name   = argv[2];

    if(!gps.open(device_name))
        return 1;

    // gps.reset(true);
    char const* solutioNames[] = {
        "NONE",
        "AUTONOMOUS",
        "DIFFERENTIAL",
        "UNUSED",
        "RTK_FIXED",
        "RTK_FLOAT"
    };

    if (!gps.setGLONASSTracking(true))
    {
        cerr << "could not enable GLONASS tracking" << endl;
        return 1;
    }
    gps.setPeriodicData(port_name, 1);
    cout << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cout << setw(10) << fields[i] << " ";
    cout << endl;

    DFKI::Time last_update;

    while(true)
    {
        gps.collectPeriodicData();
        if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
        {
            last_update = gps.position.timestamp;

	    cout
		<< setw(10) << gps.position.timestamp.toMilliseconds() << " "
		<< setw(8) << gps.position.longitude << " "
		<< setw(8) << gps.position.latitude << " "
		<< setw(8) << gps.position.altitude << " "
		<< setw(8) << gps.errors.deviationLongitude << " "
		<< setw(8) << gps.errors.deviationLatitude << " "
		<< setw(8) << gps.errors.deviationAltitude << " "
                << setw(10) << solutioNames[gps.position.positionType] << " "
                << setw(5) << gps.position.noOfSatellites << " ";

            int sat_count = gps.satellites.size();
            int counts[3] = { 0, 0, 0 };
            for (int i = 0; i < sat_count; ++i)
                counts[gps.satellites[i].getConstellation()]++;

            cout 
                << setw(5) << counts[0] << " "
                << setw(5) << counts[1] << " "
                << setw(5) << counts[2] << endl;
        }
    }
    gps.close();

    return 0;
}

