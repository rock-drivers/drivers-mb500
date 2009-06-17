#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>

using namespace std;

int main (int argc, const char** argv){
	DGPS gps;
	if( gps.open("/dev/ttyUSB0")) cout<<"init"<<endl;

	gps.getGST();

	gps.getGGA();

	GPSSatelliteInfo test = gps.getGSV();
	cout << test.sat[5].SNR <<endl;

	gps.close();

	return 0;
}
