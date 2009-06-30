#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>

using namespace std;

int main (int argc, const char** argv){
	DGPS gps;
	if( gps.open("/dev/ttyUSB0")) cout<<"init"<<endl;

	//gps.getGST();

	//gps.getGGA();

	//gps::SatelliteInfo test = gps.getGSV();
	//cout << test.sat[5].SNR <<endl;

	gps.setPeriodicData();
	while(1) gps.collectPeriodicData();


	gps.close();

	return 0;
}


/*
 *
 *
 *
 * /// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//    getFileDescriptorActivity()->watch(m_driver.getFileDescriptor());
//     return true;
// }
// bool Task::startHook()
// {
//     return true;
// }

// void Task::updateHook()
// {
// }

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }

~
~
 */
