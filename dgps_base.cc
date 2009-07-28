#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;

static const int AVERAGING_TIME = 10;
int main (int argc, const char** argv){
    DGPS gps;

    if (argc != 4)
    {
        cerr << "usage: dgps_test device_name port output_device" << endl;
        return 1;
    }

    string device_name     = argv[1];
    string current_port    = argv[2];
    string output_device   = argv[3];

    if(!gps.open(device_name))
        return 1;

    // gps.reset(true);
    char const* solutionNames[] = {
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
    gps.setPeriodicData(current_port, 1);
    cout << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cout << setw(10) << fields[i] << " ";
    cout << endl;

    DFKI::Time last_update, first_solution;
    while(true)
    {
        gps.collectPeriodicData();
        if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
        {
	    if (first_solution.isNull())
	    {
		first_solution = gps.position.timestamp;
		cout << "first solution found, now waiting " << AVERAGING_TIME << " seconds." << endl;
	    }

            last_update = gps.position.timestamp;

	    cout
		<< setw(10) << gps.position.timestamp.toMilliseconds() << " "
		<< setw(8) << gps.position.longitude << " "
		<< setw(8) << gps.position.latitude << " "
		<< setw(8) << gps.position.altitude << " "
		<< setw(8) << gps.errors.deviationLongitude << " "
		<< setw(8) << gps.errors.deviationLatitude << " "
		<< setw(8) << gps.errors.deviationAltitude << " "
                << setw(10) << solutionNames[gps.position.positionType] << " "
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

	if (!first_solution.isNull() && (gps.position.timestamp - first_solution) > DFKI::Time(AVERAGING_TIME))
	{
	    cout << "now setting base station position" << endl;
	    break;
	}
    }

    gps.stopPeriodicData();
    gps.setPosition(gps.position.latitude, gps.position.longitude, gps.position.altitude);
    gps.setRTKBase(current_port);
    char buffer[1024];

    int correction_output;
    if (output_device == "-")
	correction_output = fileno(stdout);
    else
       	correction_output = ::open(output_device.c_str(), O_WRONLY | O_CREAT);
    
    while(true)
    {
	int rd = read(gps.getFileDescriptor(), buffer, 1024);
	if (rd > 0)
	    write(correction_output, buffer, rd);
	else
	    usleep(500000);
    }
    gps.close();

    return 0;
}

