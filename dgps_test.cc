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

    gps.setProcessingRate(1);
    gps.setPeriodicData(port_name, 1);
    cout << "DGPS board initialized" << endl;
    DGPS::displayHeader(cout);

    DFKI::Time last_update;
    while(true)
    {
	try {
		gps.collectPeriodicData();
		if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
		{
		    last_update = gps.position.timestamp;
		    DGPS::display(cout, gps) << endl;

		}
	} 
        catch(timeout_error) {}
    }
    gps.close();

    return 0;
}

