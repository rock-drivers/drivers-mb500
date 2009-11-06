#include "dgps.hh"

#include <math.h>
#include <string.h>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <boost/lexical_cast.hpp>

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

DGPS::DGPS() : IODriver(2048), m_period(1000)
{
}

DGPS::~DGPS()
{
    if (isValid()) close();
}

bool DGPS::open(const string& filename)
{
    if( !IODriver::openSerial(filename, 115200))
    {
        cerr << "dgps/mb500: cannot open " << filename << " at 115200 bauds" << endl;
        return false;
    }

    int fd = getFileDescriptor();

    // Set up the serial line
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8; // data bits = 8bit
    if (tcsetattr(fd,TCSADRAIN,&tio)!=0)
    {
        cerr << "dgps/mb500: cannot set serial line properties" << endl;
        return false;
    }

    stopPeriodicData();
    stopRTKBase();
    resetStoredPosition();
    setGLONASSTracking(true);
    setCodeCorrelatorMode(STROBE_CORRELATOR);

    return true;
}

bool DGPS::openBase(std::string const& device_name)
{
    if (!open(device_name))
        return false;

    return setReceiverDynamics(STATIC);
}

bool DGPS::openRover(std::string const& device_name)
{
    if (!open(device_name))
        return false;

    return setReceiverDynamics(ADAPTIVE);
}

void DGPS::reset(bool cold_start)
{
    if (cold_start)
        write("$PASHS,INI,9,9,1\r\n", 1000);
    else
        write("$PASHS,INI,9,9,5\r\n", 1000);

    stopPeriodicData();
}

bool DGPS::stopPeriodicData()
{
    if(! setNMEALL("A", false)) return false;
    if(! setNMEALL("B", false)) return false;
    if(! setNMEALL("C", false)) return false;
    return true;
}

bool DGPS::close()
{
    IODriver::close();
    return true;
}

bool DGPS::setRTKInputPort(string const& port_name)
{
    stringstream aux;
    aux << "$PASHS,DIF,PRT," << port_name << ",RT3\r\n";
    write(aux.str(), 1000);
    return verifyAcknowledge();
}

string DGPS::read(int timeout)
{
    char buffer[MAX_PACKET_SIZE];
    size_t packet_size = readPacket(reinterpret_cast<uint8_t *>( buffer), MAX_PACKET_SIZE, timeout);
    return string(buffer, packet_size);
}

void DGPS::dumpStatus()
{
    write("$PASHQ,PAR\r\n", 1000);

    char buffer[1024];
    while(true)
    {
	usleep(100000);
	int rd = ::read(getFileDescriptor(), buffer, 1024);
	if (rd == -1)
	    break;
	if (rd > 0)
	    cout << string(buffer, rd);
    }
}
void DGPS::dumpAlmanac()
{
    write("$PASHQ,ALM\r\n", 1000);
    while(true)
    {
        string msg = read(10);
        if (msg.find("ALM") != string::npos)
            cout << msg << endl;
        else
            throw std::runtime_error("wrong reply in dumpAlmanac");
    }
}

void DGPS::dumpSatellites()
{
    write("$PASHQ,GSV\r\n", 1000);
    try
    {
        while(true) {
            std::cerr << read(10000) << std::endl;
        }
    } catch(std::runtime_error) {}
}

void DGPS::writeCorrectionData(char const* data, size_t size, int timeout)
{
    try {
        IODriver::writePacket(reinterpret_cast <uint8_t const*>(data), size, timeout);
    }
    catch(...)
    { throw std::runtime_error("dgps/mb500: error writing correction data"); }
}

void DGPS::write(const string& command, int timeout)
{
    size_t cmd_size = command.length();
    try {
        IODriver::writePacket(reinterpret_cast <uint8_t const*>(command.c_str()), cmd_size, timeout);
    }
    catch(...)
    { throw std::runtime_error("dgps/mb500: error sending command"); }
}


int DGPS::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    if(buffer[0] == '$')
    {
        for(size_t i = 1; i < buffer_size; ++i)
        {
            if( buffer[i] == '\n')
            {
                // do a bit more validation, we should have a checksum
                // at the end of the message

                // Minimal message is $*FF\r\n
                if (i < 5)
                    return -(i + 1);
                else if (buffer[i - 4] != '*')
                    return -(i + 1);

                // TODO: verify the checksum
                return i + 1;
            }
            else if (buffer[i] == '$')
            {
                // there seem to be a truncated packet, drop it
                return -i;
            }
        }

        return 0;
    }

    for(size_t i = 1; i < buffer_size; ++i)
    {
        if( buffer[i] == '$') return -i;
    }
    return -buffer_size;
}

bool DGPS::setFastRTK(bool setting)
{
    if(setting) write("$PASHS,CPD,FST,ON\r\n", 1000);
    else write("$PASHS,CPD,FST,OFF\r\n", 1000);
    return verifyAcknowledge();
}

int setRTKBaseRTCM2(std::ostream& io, string const& port_name)
{
    io << "$PASHS,RT2,18," << port_name << ",ON,1\r\n";
    io << "$PASHS,RT2,19," << port_name << ",ON,1\r\n";
    io << "$PASHS,RT2,24," << port_name << ",ON,13\r\n";
    io << "$PASHS,RT2,23," << port_name << ",ON,31\r\n";
    return 4;
}
int setRTKBaseRTCM3(std::ostream& io, string const& port_name)
{
    io << "$PASHS,RT3,1004," << port_name << ",ON,1\r\n";
    io << "$PASHS,RT3,1012," << port_name << ",ON,1\r\n";
    io << "$PASHS,RT3,1006," << port_name << ",ON,13\r\n";
    io << "$PASHS,RT3,1033," << port_name << ",ON,31\r\n";
    return 4;
}
int setRTKBaseATOM(std::ostream& io, string const& port_name)
{
    io << "$PASHS,ATM,COR," << port_name << ",ON,0.2\r\n";
    io << "$PASHS,ATM,MES," << port_name << ",ON,0.2\r\n";
    io << "$PASHS,ATM,PVT," << port_name << ",ON,13\r\n";
    io << "$PASHS,ATM,ATR," << port_name << ",ON,31\r\n";
    return 4;
}

bool DGPS::setRTKBase(string port_name)
{
    stringstream aux;
    int count = setRTKBaseRTCM3(aux, port_name);
    write(aux.str(), 1000);
    for (int i = 0; i < count; ++i)
    {
        if (!  verifyAcknowledge())
            return false;
    }
    return true;
}

void DGPS::stopRTKBase()
{
    write("$PASHS,RT2,ALL,A,OFF\r\n", 1000);
    verifyAcknowledge();
    write("$PASHS,RT2,ALL,B,OFF\r\n", 1000);
    verifyAcknowledge();
    write("$PASHS,RT2,ALL,C,OFF\r\n", 1000);
    verifyAcknowledge();
    write("$PASHS,RT3,ALL,A,OFF\r\n", 1000);
    verifyAcknowledge();
    write("$PASHS,RT3,ALL,B,OFF\r\n", 1000);
    verifyAcknowledge();
    write("$PASHS,RT3,ALL,C,OFF\r\n", 1000);
    verifyAcknowledge();
}

bool DGPS::setRTKReset()
{
    write("$PASHS,CPD,RST\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setCodeCorrelatorMode(CORRELATOR_MODE mode)
{
    char setting;
    if (mode == EDGE_CORRELATOR)
        setting = 'E';
    else
        setting = 'S';

    write(string("$PASHS,CRR,") + setting + "\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setReceiverDynamics(DYNAMICS_MODE setting)
{
    stringstream aux;
    aux << setting;
    write("$PASHS,DYN," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::resetStoredPosition()
{
    write("$PASHS,POS,MOV\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setPositionFromCurrent()
{
    write("$PASHS,POS,CUR\r\n", 1000);
    return verifyAcknowledge();
}

static double deg2magellan(double value)
{
    double deg     = static_cast<int>(value);
    double decimal = value - deg;
    return deg * 100 + decimal * 60;
}

bool DGPS::setPosition(double latitude, double longitude, double height)
{
    stringstream aux;
    aux << "$PASHS,POS,"
	<< setprecision(7) << fixed << deg2magellan(fabs(latitude))  << "," << (latitude > 0 ? 'N' : 'S') << ","
	<< setprecision(7) << fixed << deg2magellan(fabs(longitude)) << "," << (longitude > 0 ? 'W' : 'E') << ","
	<< setprecision(4) << fixed << height
	<< "\r\n";
    write(aux.str(), 1000);
    return verifyAcknowledge();
}

bool DGPS::setKnownPointInit(double latitude, string NorS, double longitude, string EorW, double height, double accLat, double accLon, double accAlt, string posAttribute)
{
    stringstream aux;
    aux << latitude << "," << NorS << "," << longitude << "," << EorW << "," << height << "," << accLat << "," << accLon << "," << accAlt << "," << posAttribute;
    write("$PASHS,KPI," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setGLONASSTracking(bool setting)
{
    if(setting) write("$PASHS,GLO,ON\r\n",1000);
    else write("$PASHS,GLO,OFF\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setSBASTracking(bool setting)
{
    if(setting) write("$PASHS,SBAS,ON\r\n",1000);
    else write("$PASHS,SBAS,OFF\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setCodeMeasurementSmoothing(int d1, int d2, int d3)
{
    stringstream aux;
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
        write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
    }

    else if(d1 >= 0 && d1 <= 100 && d2 == NONE && d3 >= 0 && d3 <= 3600) {
        aux << d1 << ",," << d3;
        write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
    }

    else if(d1 == NONE && d2 == NONE && d3 >= 0 && d3 <= 3600) {
        aux << ",," << d3;
        write("$PASHS,SMI," + aux.str() + "\r\n", 1000);
    }
    else {
        cerr<<"Arguments invalid"<<endl;
        return false;
    }

    return verifyAcknowledge();
}

bool DGPS::setNMEA(string command, string port, bool onOff, double outputRate)
{
    stringstream aux;

    string rate;
    if (fabs(outputRate - 0.1) < 0.01)
        rate = "0.1";
    else if (fabs(outputRate - 0.2) < 0.01)
        rate = "0.2";
    else if (fabs(outputRate - 0.5) < 0.01)
        rate = "0.5";
    else
        rate = boost::lexical_cast<string>(static_cast<int>(outputRate));

    if(onOff) 	aux << command << "," << port << ",ON," << rate;
    else 	aux << command << "," << port << ",OFF," << rate;
    write("$PASHS,NME," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::setPeriodicData(std::string const& port, double period)
{
    m_period = period * 1000;
    if(! setNMEA("GGA", port, true, period)) return 0;
    if(! setNMEA("GST", port, true, period)) return 0;
    if(! setNMEA("GSV", port, true, period)) return 0;
    return 1;
}

void DGPS::collectPeriodicData()
{
    string message;
    try { message = read(100); }
    catch(timeout_error)
    { return; }

    if( message.find("$GPGGA,") == 0 )
        this->position = interpretInfo(message);
    else if( message.find("$PASHR,VEC,") == 0 )
        cerr << message << endl;
    else if( message.find("$GPGST,") == 0 || message.find("$GLGST,") == 0 || message.find("$GNGST,") == 0)
        this->errors = interpretErrors(message);
    else if( message.find("$GPGSV,") == 0 || message.find("$GLGSV,") == 0)
    {
        if (interpretSatelliteInfo(tempSatellites, message))
            satellites = tempSatellites;
    }
}

bool DGPS::setNMEALL(string port, bool onOff)
{
    stringstream aux;
    if (onOff) aux << port <<",ON";
    else aux << port << ",OFF";
    write("$PASHS,NME,ALL," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge();
}

bool DGPS::verifyAcknowledge()
{
    string message;
    do
    {
        message = read(1000);
    }
    while( message.find("$PASHR,NAK") != 0 && message.find("$PASHR,ACK") != 0);
    if( message.find("$PASHR,NAK") == 0) {
        cerr << "dpgs/mb500: command not acknowledged" << endl;
        return false;
    }
    else if(message.find("$PASHR,ACK") == 0) return true;
    return false;
}


Errors DGPS::getGST(string port)
{
    if (port!= "") port = "," + port;
    write("$PASHQ,GST" + port + "\r\n", 1000);

    string result = read(1000);
    if( result.find("$PASHR,NAK") == 0) {
        cerr<<"Command not acknowledged"<<endl;
        throw runtime_error("Command not acknowledged");
    }
    return interpretErrors(result);
}

Position DGPS::getGGA(string port)
{
    string result;
    if (port!= "") port = "," + port;
    write("$PASHQ,GGA" + port + "\r\n", 1000);

    result = read(1000);
    if( result.find("$PASHR,NAK") == 0) {
        cerr<<"Command not acknowledged"<<endl;
        throw runtime_error("Command not acknowledged");
    }

    return interpretInfo(result);
}

SatelliteInfo DGPS::getGSV(string port)
{
    string msg;
    if (port!= "") port = "," + port;
    write("$PASHQ,GSV" + port + "\r\n", 1000);

    SatelliteInfo data;
    while(true)
    {
        msg = read(1000);
        if( msg.find("$PASHR,NAK") == 0) {
            cerr<<"Command not acknowledged"<<endl;
            throw runtime_error("Command not acknowledged");
        }
        else if( msg.find("$GPGSV,") != 0 && msg.find("$GLGSV,") != 0)
        {
            if (interpretSatelliteInfo(data, msg))
                return data;
        }
    }
}

Errors DGPS::interpretErrors(string const& result)
{
    Errors data;

    if( result.find("$GPGST,") == 0 || result.find("$GLGST,") == 0 || result.find("$GNGST,") == 0) {
        int pos = result.find_first_of(",", 0);

        int pos2 = result.find_first_of(",", pos+1);
        data.timestamp = interpretTime(string(result, pos+1, pos2-pos-1));

        pos = result.find_first_of(",", pos2+1);
        //float f2 = atof(string(result, pos2+1, pos - pos2 - 1).c_str());

        pos2 = result.find_first_of(",", pos+1);
        //float f3 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

        pos = result.find_first_of(",", pos2+1);
        //float f4 = atof(string(result, pos2+1, pos - pos2 - 1).c_str());

        pos2 = result.find_first_of(",", pos+1);
        //float f5 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

        pos = result.find_first_of(",", pos2+1);
        data.deviationLatitude = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

        pos2 = result.find_first_of(",", pos+1);
        data.deviationLongitude = atof( string(result, pos+1, pos2 - pos - 1).c_str());

        pos = result.find_first_of(",", pos2+1);
        data.deviationAltitude = atof( string(result, pos2+1, pos - pos2 -1).c_str());
    }
    return data;
}

bool DGPS::interpretSatelliteInfo(SatelliteInfo& data, string const& result)
{
    if( result.find("$GPGSV,") != 0 && result.find("$GLGSV,") != 0)
        throw std::runtime_error("wrong message given to interpretSatelliteInfo");

    int pos = result.find_first_of(",", 7);
    int msg_count = atoi( string(result, 7, pos-7).c_str());

    int pos2 = result.find_first_of(",", pos+1);
    int msg_number = atoi( string(result, pos+1, pos2-pos-1).c_str());

    pos = result.find_first_of(",", pos2+1);
    int sat_count = atoi(string(result, pos2+1, pos - pos2 - 1).c_str());

    if (msg_number == 1 && result.find("$GPGSV") == 0)
        data.clear();

    int field_count;
    if (msg_number != msg_count)
        field_count = 4;
    else
        field_count = sat_count % 5;

    Satellite sat;
    for(int i=0; i<field_count; ++i) {
        pos2 = result.find_first_of(",", pos+1);
        sat.PRN = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

        pos = result.find_first_of(",", pos2+1);
        sat.elevation = atoi( string(result, pos2+1, pos - pos2 - 1).c_str());

        pos2 = result.find_first_of(",", pos+1);
        sat.azimuth = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

        pos = result.find_first_of(",", pos2+1);
        sat.SNR = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

        data.push_back(sat);
    }
    return msg_number == msg_count;
}

Position DGPS::interpretInfo(string const& result)
{
    Position data;

    double m1, m2, m4, f8, f9, f10, f11;
    string c3, c5;
    int d6, d7, d12;
    if( !result.find("$GPGGA,") == 0)
        throw std::runtime_error("invalid message in interpretInfo");

    int pos = result.find_first_of(",", 7);
    data.timestamp = interpretTime(string(result, 7, pos-7));

    int pos2 = result.find_first_of(",", pos+1);
    data.latitude = atof( string(result, pos+1, pos2-pos-1).c_str());
    double minutes = fmod(data.latitude, 100);
    data.latitude = static_cast<int>(data.latitude / 100) + minutes / 60.0;

    pos = result.find_first_of(",", pos2+1);
    if (result[pos2 + 1] == 'S') data.latitude = -data.latitude;

    pos2 = result.find_first_of(",", pos+1);
    data.longitude = atof( string(result, pos+1, pos2 - pos - 1).c_str());
    minutes = fmod(data.longitude, 100);
    data.longitude = static_cast<int>(data.longitude / 100) + minutes / 60.0;

    pos = result.find_first_of(",", pos2+1);
    if (result[pos2 + 1] == 'W') data.longitude = -data.longitude;

    pos2 = result.find_first_of(",", pos+1);
    int position_type = atoi( string(result, pos+1, pos2 - pos - 1).c_str());
    if (position_type < 0 || position_type > 5)
        position_type = INVALID;
    data.positionType = static_cast<GPS_SOLUTION_TYPES>(position_type);

    pos = result.find_first_of(",", pos2+1);
    data.noOfSatellites = atoi( string(result, pos2+1, pos - pos2 - 1).c_str());

    pos2 = result.find_first_of(",", pos+1);
    f8 = atof( string(result, pos+1, pos2 - pos - 1).c_str());

    pos = result.find_first_of(",", pos2+1);
    data.altitude = atof( string(result, pos2+1, pos - pos2 -1).c_str());
    if( result[pos+1] == 'M') pos += 2;

    pos2 = result.find_first_of(",", pos+1);
    data.geoidalSeparation = atof( string(result, pos+1, pos - pos - 1).c_str());
    if( result[pos2+1] == 'M') pos2 += 2;

    pos = result.find_first_of(",", pos2+1);
    data.ageOfDifferentialCorrections = atof( string(result, pos2+1, pos - pos2 - 1).c_str());

    pos2 = result.find_first_of(".", pos+1);
    d12 = atoi( string(result, pos+1, pos2 - pos - 1).c_str());

    return data;
}

DFKI::Time DGPS::interpretTime(std::string const& time)
{
    float gps_time   = atof(time.c_str());
    int integer_part = gps_time;
    int microsecs = (gps_time - integer_part) * 1000000;

    // Get current UTC time broken down in day, h, m, s
    time_t utc_epoch = ::time(NULL);
    tm utc_hms;
    gmtime_r(&utc_epoch, &utc_hms);

    // Replace hours minutes and seconds by the GPS values
    utc_hms.tm_hour = integer_part / 10000;
    utc_hms.tm_min  = (integer_part / 100) % 100;
    utc_hms.tm_sec  = (integer_part % 100);

    // And convert it back to seconds since epoch
    time_t gps_epoch = timegm(&utc_hms);
    DFKI::Time result = DFKI::Time(gps_epoch, microsecs);
    return result;
}

char const* solutionNames[] = {
    "NONE",
    "AUTONOMOUS",
    "DIFFERENTIAL",
    "UNUSED",
    "RTK_FIXED",
    "RTK_FLOAT"
};
static const int MAX_SOLUTION_ID = 5;


std::ostream& DGPS::display(std::ostream& io, DGPS const& driver)
{
    return display(io, driver.position, driver.errors, driver.satellites);
}

std::ostream& DGPS::display(std::ostream& io, gps::Position const& pos, gps::Errors const& errors, gps::SatelliteInfo const& satellites)
{
    time_t time_secs = pos.timestamp.toSeconds();
    time_t time_msecs = pos.timestamp.toMilliseconds();

    io
        << setw(10) << ctime(&time_secs) << "." << setw(3) << time_msecs << " "
        << setprecision(10) << fixed << setw(15) << pos.longitude << " "
        << setw(15) << pos.latitude << " "
        << setprecision(2) << setw(8) << pos.altitude << " "
        << setprecision(3) << setw(8) << errors.deviationLongitude << " "
        << setprecision(3) << setw(8) << errors.deviationLatitude << " "
        << setprecision(3) << setw(8) << errors.deviationAltitude << " ";

    if (pos.positionType > MAX_SOLUTION_ID)
        io << setw(10) << "UNDEFINED=" << (int)pos.positionType << " ";
    else
        io << setw(10) << solutionNames[pos.positionType] << " ";

    io << setw(5) << pos.noOfSatellites << " ";

    int sat_count = satellites.size();
    int counts[3] = { 0, 0, 0 };
    for (int i = 0; i < sat_count; ++i)
        counts[satellites[i].getConstellation()]++;

    io 
        << setw(5) << counts[0] << " "
        << setw(5) << counts[1] << " "
        << setw(5) << counts[2] << " "
        << setw(5);

    io << pos.ageOfDifferentialCorrections;

    return io;
}

