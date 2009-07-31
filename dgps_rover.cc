#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>
#include <fcntl.h>

using namespace std;

int main (int argc, const char** argv){
    DGPS gps;

    if (argc != 4)
    {
        cerr << "usage: dgps_test device_name port_name correction_device" << endl;
        return 1;
    }

    string device_name = argv[1];
    string port_name   = argv[2];
    string correction_device = argv[3];

    if(!gps.open(device_name))
        return 1;

    gps.dumpStatus();

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
    if (!gps.setRTKInputPort(port_name))
    {
        cerr << "could not setup correction input" << endl;
        return 1;
    }
    gps.setRTKOutputMode(false);
    gps.setPeriodicData(port_name, 1);
    cout << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cout << setw(10) << fields[i] << " ";
    cout << endl;

    DFKI::Time last_update;

    int correction_input;
    if (correction_device == "-")
    {
        correction_input = dup(fileno(stdin));
        long fd_flags = fcntl(correction_input, F_GETFL);
        fcntl(correction_input, F_SETFL, fd_flags | O_NONBLOCK);
    }
    else
        correction_input = open(correction_device.c_str(), O_NONBLOCK | O_RDONLY);

    int correction_log = open("corrections.log", O_WRONLY | O_CREAT | O_TRUNC);

    char buffer[1024];
    while(true)
    {
        int rd;
        while ( (rd = read(correction_input, buffer, 1024)) > 0)
        {
            cerr << "DEBUG: received " << rd << " bytes of correction stream" << endl;
            write(correction_log, buffer, rd);
            write(gps.getFileDescriptor(), buffer, rd);
            usleep(100000);
        }

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
                << setw(5) << counts[2] << " ";

            cout << setw(5) << gps.position.ageOfDifferentialCorrections << endl;
        }
    }
    gps.close();

    return 0;
}

