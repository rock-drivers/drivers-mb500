#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>
#include <boost/lexical_cast.hpp>

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;

int openSocket(std::string const& hostname, std::string const& port)
{
    // Open the UDP socket (for later)
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */

    struct addrinfo *result;
    int ret = getaddrinfo(hostname.c_str(), port.c_str(), &hints, &result);
    if (ret > 0)
    {
        fprintf(stderr, "failed to resolve %s: %s\n", hostname.c_str(), gai_strerror(ret));
        return -1;
    }

    int sfd = -1;
    struct addrinfo *rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype,
                rp->ai_protocol);
        if (sfd == -1)
            continue;

        if (connect(sfd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;                  /* Success */

        close(sfd);
    }
    freeaddrinfo(result);

    if (rp == NULL)
    {
        cerr << "failed to create socket" << endl;
        return -1;
    }

    return sfd;
}

static const int AVERAGING_TIME     = 2;
static const int AVERAGING_SAMPLING = 1;
int main (int argc, const char** argv){
    DGPS gps;

    if (argc != 5)
    {
        cerr << "usage: dgps_test device_name port target_host target_port" << endl;
        return 1;
    }

    string device_name     = argv[1];
    string current_port    = argv[2];
    string target_host     = argv[3];
    string target_port     = argv[4];

    struct stat file_stat;
    int diff_io = -1;
    if (target_host == "-")
    {
        diff_io = dup(fileno(stdout));
        cerr << "outputting correction data to standard output" << endl;
    }
    else if (stat(target_host.c_str(), &file_stat) != -1)
    {
        diff_io = IODriver::openSerialIO(target_host, boost::lexical_cast<int>(target_port));
        cerr << "outputting correction data to serial port " << target_host << ", baud rate is " << target_port << endl;
    }
    else
    {
        diff_io = openSocket(target_host, target_port);
        if (diff_io == -1)
        {
            cerr << "cannot open target host/port" << endl;
            return 1;
        }
        cerr << "outputting correction data to " << target_host << ":" << target_port << endl;
    }

    if(!gps.openBase(device_name))
        return 1;

    if (!gps.setGLONASSTracking(true))
    {
        cerr << "could not enable GLONASS tracking" << endl;
        return 1;
    }
    if (!gps.setCodeCorrelatorMode(DGPS::EDGE_CORRELATOR))
    {
        cerr << "could not set the code correlator" << endl;
        return 1;
    }
    if (!gps.setReceiverDynamics(DGPS::STATIC))
    {
        cerr << "could not set the receiver dynamics" << endl;
        return 1;
    }

    gps.setPeriodicData(current_port, AVERAGING_SAMPLING);
    cerr << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cerr << setw(10) << fields[i] << " ";
    cerr << endl;

    DFKI::Time last_update, first_solution;

    size_t count = 0;
    double pos[3] = { 0, 0, 0 };
    while(true)
    {
        gps.collectPeriodicData();
        if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
        {
	    if (gps.position.positionType != gps::NO_SOLUTION)
	    {
                if (first_solution.isNull())
                {
                    first_solution = gps.position.timestamp;
                    cerr << "first solution found, now waiting " << AVERAGING_TIME << " seconds." << endl;
                }
                pos[0] += gps.position.latitude;
                pos[1] += gps.position.longitude;
                pos[2] += gps.position.altitude;
                ++count;
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
    pos[0] /= count; pos[1] /= count; pos[2] /= count;
    cerr << "setting position to: lat=" << pos[0] << ", long=" << pos[1] << ", alt=" << pos[2] << endl;
    //gps.setPosition(pos[0], pos[1], pos[2]);
    gps.setPositionFromCurrent();
    gps.setRTKBase(current_port);
    char buffer[1024];

    while(true)
    {
	int rd = read(gps.getFileDescriptor(), buffer, 1024);
        if (rd > 0)
        {
            int written = write(diff_io, buffer, rd);
            if (written == -1)
            {
                // if ECONNREFUSED, there's nobody at the other end
                if (errno != ECONNREFUSED)
                    cerr << "error during write: " << strerror(errno) << endl;
            }
            else if (written != rd)
                cerr << "partial write ! (written " << written << ", needed " << rd << ")" << endl;
            else
                cerr << "written " << rd << " bytes" << endl;
        }
        usleep(50000);
    }

    return 0;
}

