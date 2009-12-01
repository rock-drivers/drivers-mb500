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

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace std;
using namespace gps;
using namespace boost;

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

bool DGPS::setUserDynamics(int h_vel, int h_acc, int v_vec, int v_acc)
{
    write("$PASHS,UDP," +
	    boost::lexical_cast<string>(h_vel) + "," +
	    boost::lexical_cast<string>(h_acc) + "," +
	    boost::lexical_cast<string>(v_vec) + "," +
	    boost::lexical_cast<string>(v_acc) + "\r\n", 1000);
    return verifyAcknowledge("USER DYNAMICS");
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
    return verifyAcknowledge("RTK INPUT PORT");
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
    io << "$PASHS,RT3,1004," << port_name << ",ON,0.5\r\n";
    io << "$PASHS,RT3,1012," << port_name << ",ON,0.5\r\n";
    io << "$PASHS,RT3,1006," << port_name << ",ON,5\r\n";
    io << "$PASHS,RT3,1033," << port_name << ",ON,10\r\n";
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
    verifyAcknowledge("RT2,A,OFF");
    write("$PASHS,RT2,ALL,B,OFF\r\n", 1000);
    verifyAcknowledge("RT2,B,OFF");
    write("$PASHS,RT2,ALL,C,OFF\r\n", 1000);
    verifyAcknowledge("RT2,C,OFF");
    write("$PASHS,RT3,ALL,A,OFF\r\n", 1000);
    verifyAcknowledge("RT2,A,OFF");
    write("$PASHS,RT3,ALL,B,OFF\r\n", 1000);
    verifyAcknowledge("RT2,B,OFF");
    write("$PASHS,RT3,ALL,C,OFF\r\n", 1000);
    verifyAcknowledge("RT2,C,OFF");
}

bool DGPS::setRTKReset()
{
    write("$PASHS,CPD,RST\r\n", 1000);
    return verifyAcknowledge("RTK RESET");
}

bool DGPS::setCodeCorrelatorMode(CORRELATOR_MODE mode)
{
    char setting;
    if (mode == EDGE_CORRELATOR)
        setting = 'E';
    else
        setting = 'S';

    write(string("$PASHS,CRR,") + setting + "\r\n", 1000);
    return verifyAcknowledge("CODE CORRELATOR " + setting);
}

bool DGPS::setReceiverDynamics(DYNAMICS_MODEL setting)
{
    stringstream aux;
    aux << setting;
    write("$PASHS,DYN," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge("RECEIVER DYNAMICS " + setting);
}

bool DGPS::resetStoredPosition()
{
    write("$PASHS,POS,MOV\r\n", 1000);
    return verifyAcknowledge("RESET STORED POSITION");
}

bool DGPS::setPositionFromCurrent()
{
    write("$PASHS,POS,CUR\r\n", 1000);
    return verifyAcknowledge("SET POSITION FROM CURRENT");
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
    return verifyAcknowledge("SET CURRENT POSITION");
}

bool DGPS::setKnownPointInit(double latitude, string NorS, double longitude, string EorW, double height, double accLat, double accLon, double accAlt, string posAttribute)
{
    stringstream aux;
    aux << latitude << "," << NorS << "," << longitude << "," << EorW << "," << height << "," << accLat << "," << accLon << "," << accAlt << "," << posAttribute;
    write("$PASHS,KPI," + aux.str() + "\r\n", 1000);
    return verifyAcknowledge("SET INITIAL POSITION");
}

bool DGPS::setGLONASSTracking(bool setting)
{
    if(setting) write("$PASHS,GLO,ON\r\n",1000);
    else write("$PASHS,GLO,OFF\r\n", 1000);
    return verifyAcknowledge("SET GLONASS TRACKING");
}

bool DGPS::setSBASTracking(bool setting)
{
    if(setting) write("$PASHS,SBAS,ON\r\n",1000);
    else write("$PASHS,SBAS,OFF\r\n", 1000);
    return verifyAcknowledge("SET SBAS TRACKING");
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

    return verifyAcknowledge("CODE SMOOTHING");
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
    return verifyAcknowledge("NMEA OUTPUT " + command + " " + (onOff ? "ON" : "OFF") + " " + rate);
}

bool DGPS::setPeriodicData(std::string const& port, double period)
{
    m_period = period * 1000;

    int stats_period = period;
    if (stats_period < 5)
	stats_period = 5;

    if(! setNMEA("GGA", port, true, period)) return 0;
    if(! setNMEA("GST", port, true, period)) return 0;
    if(! setNMEA("GSA", port, true, stats_period)) return 0;
    if(! setNMEA("GSV", port, true, stats_period)) return 0;
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
    else if( message.find("$GPGSA,") == 0 || message.find("$GLGSA,") == 0 || message.find("$GNGSA,") == 0 )
        this->solutionQuality = interpretQuality(message);
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
    return verifyAcknowledge("NMEA ALL");
}

bool DGPS::verifyAcknowledge(std::string const& cmd)
{
    string message;
    do
    {
        message = read(1000);
    }
    while( message.find("$PASHR,NAK") != 0 && message.find("$PASHR,ACK") != 0);
    if( message.find("$PASHR,NAK") == 0) {
        cerr << "dpgs/mb500: command " << cmd << " not acknowledged" << endl;
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

SolutionQuality DGPS::interpretQuality(string const& message)
{
    if( message.find("$GPGSA,") != 0 && message.find("$GLGSA,") != 0 && message.find("$GNGSA,") != 0)
        throw std::runtime_error("wrong message given to interpretErrors");

    vector<string> fields;
    split( fields, message, is_any_of(",*") );

    SolutionQuality data;
    data.timestamp = DFKI::Time::now();
    int sat_end = fields.size() - 4;
    for (int i = 3; i < sat_end; ++i)
    {
        if (fields[i] != "")
            data.usedSatellites.push_back( atoi(fields[i].c_str()) );
    }

    data.pdop = atof(fields[sat_end].c_str());
    data.hdop = atof(fields[sat_end + 1].c_str());
    data.vdop = atof(fields[sat_end + 2].c_str());
    return data;
}

Errors DGPS::interpretErrors(string const& message)
{
    if( message.find("$GPGST,") != 0 && message.find("$GLGST,") != 0 && message.find("$GNGST,") != 0)
        throw std::runtime_error("wrong message given to interpretErrors");

    vector<string> fields;
    split( fields, message, is_any_of(",*") );

    Errors data;
    data.timestamp = interpretTime(fields[1]);
    data.deviationLatitude  = atof(fields[6].c_str());
    data.deviationLongitude = atof(fields[7].c_str());
    data.deviationAltitude  = atof(fields[8].c_str());
    return data;
}

bool DGPS::interpretSatelliteInfo(SatelliteInfo& data, string const& message)
{
    if( message.find("$GPGSV,") != 0 && message.find("$GLGSV,") != 0)
        throw std::runtime_error("wrong message given to interpretSatelliteInfo");

    vector<string> fields;
    split( fields, message, is_any_of(",*") );
    int msg_count  = atoi(fields[1].c_str());
    int msg_number = atoi(fields[2].c_str());
    int sat_count  = atoi(fields[3].c_str());

    // interpretSatelliteInfo() accumulates information, since the information
    // is spanned over multiple messages. We clear data if this is the first
    // message of a series.
    if (msg_number == 1 && message.find("$GPGSV") == 0)
    {
        data.knownSatellites.clear();
        data.timestamp = DFKI::Time::now();
    }

    // Compute the number of satellites in this message
    int field_count;
    if (msg_number != msg_count)
        field_count = 4;
    else
        field_count = sat_count - (msg_count - 1) * 4;

    Satellite sat;
    for(int i = 0; i < field_count; ++i) {
        sat.PRN       = atoi(fields[4 + i * 4].c_str());
        sat.elevation = atoi(fields[5 + i * 4].c_str());
        sat.azimuth   = atoi(fields[6 + i * 4].c_str());
        sat.SNR       = atoi(fields[7 + i * 4].c_str());

        data.knownSatellites.push_back(sat);
    }
    return (msg_number == msg_count && message.find("$GLGSV") == 0);
}

Position DGPS::interpretInfo(string const& message)
{
    if( !message.find("$GPGGA,") == 0)
        throw std::runtime_error("invalid message in interpretInfo");

    vector<string> fields;
    split( fields, message, is_any_of(",*") );

    Position data;

    data.timestamp = interpretTime(fields[1]);
    data.latitude  = interpretAngle(fields[2], fields[3] == "N");
    data.longitude = interpretAngle(fields[4], fields[5] == "E");
    int position_type = atoi(fields[6].c_str());
    if (position_type < 0 || position_type > 5)
        position_type = INVALID;
    data.positionType = static_cast<GPS_SOLUTION_TYPES>(position_type);

    data.noOfSatellites = atoi(fields[7].c_str());
    data.altitude       = atof(fields[9].c_str());
    data.geoidalSeparation = atof(fields[11].c_str());
    data.ageOfDifferentialCorrections = atof(fields[13].c_str());
    return data;
}

double DGPS::interpretAngle(std::string const& value, bool positive)
{
    double angle = atof(value.c_str());
    double minutes = fmod(angle, 100);
    angle = static_cast<int>(angle / 100) + minutes / 60.0;
    if (!positive)
        angle = -angle;
    return angle;
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
    return display(io, driver.position, driver.errors, driver.satellites, driver.solutionQuality);
}

std::ostream& DGPS::displayHeader(std::ostream& io)
{
    cout << "Time                          | Latitude      Longitude     Altitude | dLat   dLong  dAlt | Mode      PDOP    Used   Tracked | DiffAge" << std::endl;
    return io;
}

std::ostream& DGPS::display(std::ostream& io,
	gps::Position const& pos, gps::Errors const& errors,
	gps::SatelliteInfo const& satellites,
	gps::SolutionQuality const& quality)
{
    time_t time_secs = pos.timestamp.toSeconds();
    int time_msecs   = pos.timestamp.microseconds / 1000;

    char* time_string = ctime(&time_secs);
    io
        << setw(10) << string(time_string, time_string + strlen(time_string) - 1) << "." << setw(3) << setfill('0') << time_msecs << " "
	<< setfill(' ')
	<< " | "
        << setprecision(10) << fixed << setw(13) << pos.longitude << " "
        << setprecision(10) << fixed << setw(13) << pos.latitude << " "
        << setprecision(2) << setw(7) << pos.altitude << " "
	<< " | "
        << setprecision(2) << setw(5) << errors.deviationLongitude << " "
        << setprecision(2) << setw(5) << errors.deviationLatitude << " "
        << setprecision(2) << setw(5) << errors.deviationAltitude << " "
	<< " | ";

    if (pos.positionType > MAX_SOLUTION_ID)
        io << setw(10) << "UNDEFINED=" << (int)pos.positionType << " ";
    else
        io << setw(10) << solutionNames[pos.positionType] << " ";

    io << setw(4) << setprecision(1) << quality.hdop << " ";
    io << setw(2) << pos.noOfSatellites << " ";
    { // count the number of satellites in use, per constellation
	int sat_count = quality.usedSatellites.size();
	int counts[3] = { 0, 0, 0 };
	for (int i = 0; i < sat_count; ++i)
	    counts[Satellite::getConstellationFromPRN(quality.usedSatellites[i])]++;
	io << setw(2) << counts[0] << "/" << setw(2) << counts[2] << " ";
    }

    { // count the number of satellites in view, per constellation
	int sat_count = satellites.knownSatellites.size();
	int counts[3] = { 0, 0, 0 };
	for (int i = 0; i < sat_count; ++i)
	{
            Satellite const& sat = satellites.knownSatellites[i];
	    if (sat.SNR > 0)
		counts[sat.getConstellation()]++;
	}
	io << setw(2) << counts[0] << "/" << setw(2) << counts[2] << " ";
    }

    io << " | " << pos.ageOfDifferentialCorrections;

    return io;
}

