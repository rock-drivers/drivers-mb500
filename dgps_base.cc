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

    if (!gps.setGLONASSTracking(true))
    {
        cerr << "could not enable GLONASS tracking" << endl;
        return 1;
    }
    gps.setCodeCorrelatorMode(DGPS::EDGE_CORRELATOR);
    gps.setReceiverDynamics(DGPS::STATIC);

    gps.setPeriodicData(current_port, 1);
    cerr << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cerr << setw(10) << fields[i] << " ";
    cerr << endl;

    DFKI::Time last_update, first_solution;
    while(true)
    {
        gps.collectPeriodicData();
        if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
        {
	    if (gps.position.positionType != gps::NO_SOLUTION && first_solution.isNull())
	    {
		first_solution = gps.position.timestamp;
		cerr << "first solution found, now waiting " << AVERAGING_TIME << " seconds." << endl;
	    }

            last_update = gps.position.timestamp;
            DGPS::display(cerr, gps) << endl;
        }

	if (!first_solution.isNull() && (gps.position.timestamp - first_solution) > DFKI::Time(AVERAGING_TIME))
	{
	    cerr << "now setting base station position" << endl;
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
