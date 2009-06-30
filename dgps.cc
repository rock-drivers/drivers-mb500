#include "dgps.hh"

#include <map>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <termio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace gps;

DGPS::DGPS() : IODriver(2048), baudrate(19200)
{
	releaseOK = false;
}

DGPS::~DGPS()
{
	if (isValid()) close();
}

bool DGPS::open(const string& filename)
{
	if( !IODriver::openSerial(filename, 115200)) return false;

	int fd = getFileDescriptor();
    struct termios tio;
    tcgetattr(fd,&tio);
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8; // data bits = 8bit

    // Commit
    if (tcsetattr(fd,TCSADRAIN,&tio)!=0)
        return false;

	cout<<"GPS successfully initialized on "<< filename <<endl;

	return stopPeriodicData();
}

bool DGPS::stopPeriodicData()
{
	if(! setNMEALL("A", OFF)) return false;
	if(! setNMEALL("B", OFF)) return false;
	if(! setNMEALL("C", OFF)) return false;
	return true;
}

bool DGPS::close()
{
	IODriver::close();
	return true;
}

string DGPS::read(int timeout)
{
	char buffer[MAX_PACKET_SIZE];
	size_t packet_size = readPacket(reinterpret_cast<uint8_t *>( buffer), MAX_PACKET_SIZE, timeout);
	cerr << "RD: " << string(buffer, packet_size) << endl;
	return string(buffer, packet_size);
}

bool DGPS::write(const string& command, int timeout)
{
	size_t cmd_size = command.length();
	try {
		cerr << "WR: " << command << endl;
		IODriver::writePacket(reinterpret_cast <uint8_t const*>(command.c_str()), cmd_size, timeout);
		return true;
	}
	catch(...) { return false; }
}


int DGPS::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
	if(buffer[0] == '$')
	{
		for(size_t i = 1; i < buffer_size; ++i) if( buffer[i] == '\n') return i;
		return 0;
	}

	for(size_t i = 1; i < buffer_size; ++i)
	{
		if( buffer[i] == '$') return (-1) * (i);
	}
	return -buffer_size;
}

bool DGPS::setRTKOutputMode(bool setting)
{
	bool result;
	if(setting) result = write("$PASHS,CPD,FST,ON\r\n", 1000);
	else result = write("$PASHS,CPD,FST,OFF\r\n", 1000);

	if(result) return verifyAcknowledge();
	return false;
}

bool DGPS::setRTKReset()
{
	if(write("$PASHS,CPD,RST\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::setCodeCorrelatorMode(string setting)
{
	if(write("$PASHS,CRR," + setting + "\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::setReceiverDynamics(int setting)
{
	stringstream aux;
	aux << setting;
	if(write("$PASHS,DYN," + aux.str() + "\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::setKnownPointInit(double latitude, string NorS, double longitude, string EorW, double height, double accLat, double accLon, double accAlt, string posAttribute)
{
	stringstream aux;
	aux << latitude << "," << NorS << "," << longitude << "," << EorW << "," << height << "," << accLat << "," << accLon << "," << accAlt << "," << posAttribute;
	if(write("$PASHS,KPI," + aux.str() + "\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::setSBASTracking(bool setting)
{
	bool result;
	if(setting) result = write("$PASHS,SBAS,ON\r\n",1000);
	else result = write("$PASHS,SBAS,OFF\r\n", 1000);
	if(result) return verifyAcknowledge();
	return false;
}

bool DGPS::setCodeMeasurementSmoothing(int d1, int d2, int d3)
{
	stringstream aux;
	bool result;
	if(d1 >= 0 && d1 <= 100) {
		aux << d1;
		if( d2 >= 100 && d2 <= 600) {
			aux << "," << d2;
			if( d3 >= 0 && d3 <= 3600) aux << "," << d3;
			else if (d3 != NONE) {
				cerr <<"Arguments invalid"<<endl;
				return false;
			}
		}
		else if (d2 != NONE) {
			cerr <<"Arguments invalid"<<endl;
			return false;
		}
		result = write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
	}

	else if(d1 >= 0 && d1 <= 100 && d2 == NONE && d3 >= 0 && d3 <= 3600) {
		aux << d1 << ",," << d3;
		result = write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
	}

	else if(d1 == NONE && d2 == NONE && d3 >= 0 && d3 <= 3600) {
		aux << ",," << d3;
		result = write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
	}
	else {
		cerr<<"Arguments invalid"<<endl;
		return false;
	}

	if(result) return verifyAcknowledge();
	return false;
}

bool DGPS::setNMEA(string command, string port, bool onOff, double outputRate)
{
	stringstream aux;
	if(onOff) 	aux << command << "," << port << ",ON," << outputRate;
	else 	aux << command << "," << port << ",OFF," << outputRate;
	if(write("$PASHS,NME," + aux.str() + "\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::setPeriodicData()
{
	if(! setNMEA("GGA", "B", true, 1)) return 0;
	if(! setNMEA("GST", "B", true, 1)) return 0;
	if(! setNMEA("GSV", "B", true, 1)) return 0;
	return 1;
}

void DGPS::collectPeriodicData()
{
	cerr << "reading periodic data"<<endl;
	string message = read(1000);
	cerr << "done reading periodic data" <<endl;

	if( message.find("$GPGGA,") == 0 && data.info.UTCTime < data.errors.UTCTime ) data.info = interpretInfo(message);
	else if( message.find("$GPGST,") == 0 || message.find("$GLGST,") == 0 || message.find("$GNGST,") == 0) data.errors = interpretErrors(message);
	else if( message.find("$GPGSV,") == 0 || message.find("$GLGSV,") == 0) dataSatellite = interpretSatelliteInfo(message);

	if( data.info.UTCTime == data.errors.UTCTime) {
		cerr << "We have a synced data set at: "<< data.info.UTCTime <<endl;
		releaseOK = true;
	}
	else releaseOK = false;
}

bool DGPS::setNMEALL(string port, bool onOff)
{
	stringstream aux;
	if (onOff) aux << port <<",ON";
	else aux << port << ",OFF";
	if(write("$PASHS,NME,ALL," + aux.str() + "\r\n", 1000)) return verifyAcknowledge();
	return false;
}

bool DGPS::verifyAcknowledge()
{
	string message = read(1000);

	//keep on reading until you find an ACK or NAK
	while( message.find("$PASHR,NAK") != 0 && message.find("$PASHR,ACK") != 0) message = read(1000);
	cerr << "Found ACK / NAK ................................ " << endl;
	if( message.find("$PASHR,NAK") == 0) {
		cerr<<"Command not acknowledged"<<endl;
		return false;
	}
	else if(message.find("$PASHR,ACK") == 0) return true;
	return false;
}


Errors DGPS::getGST(string port)
{
	if (port!= "") port = "," + port;
	if( write("$PASHQ,GST" + port + "\r\n", 1000)) {
		string result = read(1000);
		cout<<result<<endl;
		if( result.find("$PASHR,NAK") == 0) {
			cerr<<"Command not acknowledged"<<endl;
			throw runtime_error("Command not acknowledged");
		}
		return interpretErrors(result);
	}
	else throw runtime_error("Error while sending command");
}

Info DGPS::getGGA(string port)
{
	string result;
	if (port!= "") port = "," + port;
	if( write("$PASHQ,GGA" + port + "\r\n", 1000)) {
		result = read(1000);
		cout<<result<<endl;
		if( result.find("$PASHR,NAK") == 0) {
			cerr<<"Command not acknowledged"<<endl;
			throw runtime_error("Command not acknowledged");
		}
	}
	else throw runtime_error("Error while sending command");

	return interpretInfo(result);
}

SatelliteInfo DGPS::getGSV(string port)
{
	string result;
	if (port!= "") port = "," + port;
	if( write("$PASHQ,GSV" + port + "\r\n", 1000)) {
		result = read(1000);
		cout<<result<<endl;
		if( result.find("$PASHR,NAK") == 0) {
			cerr<<"Command not acknowledged"<<endl;
			throw runtime_error("Command not acknowledged");
		}
	}
	else throw runtime_error("Error while sending command");

	return interpretSatelliteInfo(result);
}

Errors DGPS::interpretErrors(string &result)
{
	Errors data;
	double m1, f2, f3, f4, f5, f6, f7, f8;
	string header;

	if( result.find("$GPGST,") == 0 || result.find("$GLGST,") == 0 || result.find("$GNGST,") == 0) {

		int pos = result.find_first_of(",", 0);
		header = string(result, 0, pos);

		int pos2 = result.find_first_of(",", pos+1);
		m1 = atof( string(result, pos+1, pos2-pos-1).c_str());

		pos = result.find_first_of(",", pos2+1);
		f2 = atof(string(result, pos2+1, pos - pos2 - 1).c_str());

		pos2 = result.find_first_of(",", pos+1);
		f3 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		f4 = atof(string(result, pos2+1, pos - pos2 - 1).c_str());

		pos2 = result.find_first_of(",", pos+1);
		f5 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		f6 = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

		pos2 = result.find_first_of(",", pos+1);
		f7 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		f8 = atof( string(result, pos2+1, pos - pos2 -1).c_str());
	}

	cerr<< header <<endl<<m1<<endl<<f2<<endl<<f3<<endl<<f4<<endl<<f5<<endl<<f6<<endl<<f7<<endl<<f8<<endl;

	data.UTCTime = m1;
	data.deviationLatitude = f6;
	data.deviationLongitude = f7;
	data.deviationAltitude = f8;
	return data;
}

SatelliteInfo DGPS::interpretSatelliteInfo(string &result)
{
	int d1, d2, d3, d4, d5, d6;
	double f7;
	SatelliteInfo data;

	if( result.find("$GPGSV,") == 0 || result.find("$GLGSV,") == 0) {
		int pos = result.find_first_of(",", 7);
		d1 = atoi( string(result, 7, pos-7).c_str());

		int pos2 = result.find_first_of(",", pos+1);
		d2 = atoi( string(result, pos+1, pos2-pos-1).c_str());

		pos = result.find_first_of(",", pos2+1);
		d3 = atoi(string(result, pos2+1, pos - pos2 - 1).c_str());

		int noSatTot = d3, noSatLine;
		if(noSatTot >= 4) noSatLine = 4;
		else noSatLine = noSatTot;
		data.noOfSatellites = d3;
		data.sat.clear();
		Satellite sat;

		cout<<d1<<endl<<d2<<endl<<d3<<endl;
		for(int i=0; i<noSatLine; ++i) {
			pos2 = result.find_first_of(",", pos+1);
			d4 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

			pos = result.find_first_of(",", pos2+1);
			d5 = atoi( string(result, pos2+1, pos - pos2 - 1).c_str());

			pos2 = result.find_first_of(",", pos+1);
			d6 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

			pos = result.find_first_of(",", pos2+1);
			f7 = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

			sat.PRN = d4;
			sat.elevation = d5;
			sat.azimuth = d6;
			sat.SNR = f7;
			data.sat.push_back(sat);

			cout<<d4<<" "<<d5<<" "<<d6<<" "<<f7<<endl;
		}
		noSatTot -= 4;

		while(noSatTot > 0) {
			result = read(1000);
			cout<<result<<endl;


			if(noSatTot >= 4) noSatLine = 4;
			else noSatLine = noSatTot;

			int pos = result.find_first_of(",", 7);
			d1 = atoi( string(result, 7, pos-7).c_str());

			int pos2 = result.find_first_of(",", pos+1);
			d2 = atoi( string(result, pos+1, pos2-pos-1).c_str());

			pos = result.find_first_of(",", pos2+1);
			d3 = atoi(string(result, pos2+1, pos - pos2 - 1).c_str());
			cout<<d1<<endl<<d2<<endl<<d3<<endl;

			for(int i=0; i<noSatLine; ++i) {
				pos2 = result.find_first_of(",", pos+1);
				d4 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

				pos = result.find_first_of(",", pos2+1);
				d5 = atoi( string(result, pos2+1, pos - pos2 - 1).c_str());

				pos2 = result.find_first_of(",", pos+1);
				d6 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

				pos = result.find_first_of(",", pos2+1);
				f7 = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

				sat.PRN = d4;
				sat.elevation = d5;
				sat.azimuth = d6;
				sat.SNR = f7;
				data.sat.push_back(sat);

				cout<<d4<<" "<<d5<<" "<<d6<<" "<<f7<<endl;
			}
			noSatTot -= 4;
		}
	}
	return data;
}

Info DGPS::interpretInfo(string &result)
{
	Info data;

	double m1, m2, m4, f8, f9, f10, f11;
	string c3, c5;
	int d6, d7, d12;
	if( result.find("$GPGGA,") == 0) {
		int pos = result.find_first_of(",", 7);
		m1 = atof( string(result, 7, pos-7).c_str());

		int pos2 = result.find_first_of(",", pos+1);
		m2 = atof( string(result, pos+1, pos2-pos-1).c_str());

		pos = result.find_first_of(",", pos2+1);
		c3 = string(result, pos2+1, pos - pos2 - 1);
		if (c3 == "S") m2 = -m2;

		pos2 = result.find_first_of(",", pos+1);
		m4 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		c5 = string(result, pos2+1, pos - pos2 - 1);
		if (c5 == "W") m4 = -m4;

		pos2 = result.find_first_of(",", pos+1);
		d6 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		d7 = atoi( string(result, pos2+1, pos - pos2 - 1).c_str());

		pos2 = result.find_first_of(",", pos+1);
		f8 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

		pos = result.find_first_of(",", pos2+1);
		f9 = atof( string(result, pos2+1, pos - pos2 -1).c_str());
		if( result[pos+1] == 'M') pos += 2;

		pos2 = result.find_first_of(",", pos+1);
		f10 = atof( string(result, pos+1, pos - pos - 1).c_str());
		if( result[pos2+1] == 'M') pos2 += 2;

		pos = result.find_first_of(",", pos2+1);
		f11 = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

		pos2 = result.find_first_of(".", pos+1);
		d12 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

		cout<< m1 <<endl<<m2<<endl<<c3<<endl<<m4<<endl<<c5<<endl<<d6<<endl<<d7<<endl<<f8<<endl<<f9<<endl<<f10<<endl<<f11<<endl<<d12<<endl;
		data.UTCTime = m1;
		data.latitude = m2;
		data.longitude = m4;
		data.positionType = d6;
		data.noOfSatellites = d7;
		data.altitude = f9;
		data.geoidalSeparation = f10;
		data.ageOfDifferentialCorrections = f11;
	}
	return data;
}
