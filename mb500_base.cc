#include "mb500.hh"
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
using namespace gps_base;

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

static const int AVERAGING_TIME     = 10;
static const int AVERAGING_SAMPLING = 1;
int main (int argc, const char** argv){
    gps::MB500 gps;

    if (argc != 4 && argc != 5 && argc != 8)
    {
        cerr << "usage: dgps_test device_name port target_host target_port [lattitude longitude altitude]" << endl;
        return 1;
    }

    string device_name     = argv[1];
    string current_port    = argv[2];
    string target_host     = argv[3];

    struct stat file_stat;
    int diff_io = -1;
    if (target_host == "-")
    {
        diff_io = dup(fileno(stdout));
        cerr << "outputting correction data to standard output" << endl;
    }
    else if (stat(target_host.c_str(), &file_stat) != -1)
    {
	string target_port     = argv[4];
        diff_io = iodrivers_base::Driver::openSerialIO(target_host, boost::lexical_cast<int>(target_port));
        cerr << "outputting correction data to serial port " << target_host << ", baud rate is " << target_port << endl;
    }
    else
    {
	string target_port     = argv[4];
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

    if (!gps.setReceiverDynamics(gps::MB500_STATIC))
    {
        cerr << "could not set the receiver dynamics" << endl;
        return 1;
    }


    gps.setPeriodicData(current_port, AVERAGING_SAMPLING);
    cerr << "MB500 board initialized" << endl;
    gps::MB500::displayHeader(cerr);
    base::Time last_update, first_solution;

    if(argc == 8) {
	double pos[3] = { 0, 0, 0 };
	pos[0] = boost::lexical_cast<double>(argv[5]);
	pos[1] = boost::lexical_cast<double>(argv[6]);
	pos[2] = boost::lexical_cast<double>(argv[7]);
	cerr << "setting position to: "
            << "lat  " << setprecision(10) << fixed << pos[0] << endl
            << "long " << setprecision(10) << fixed << pos[1] << endl
            << "alt  " << setprecision(2)  << fixed << pos[2] << endl;

        base::Time current_timestamp = gps.position.time;
	gps.setPosition(pos[0], pos[1], pos[2]);
        while (true)
        {
            gps.collectPeriodicData();
            if (current_timestamp != gps.position.time)
                break;
        }

	cerr << "board reports: " << endl
            << "lat  " << setprecision(10) << fixed << gps.position.latitude << endl
            << "long " << setprecision(10) << fixed << gps.position.longitude << endl
            << "alt  " << setprecision(2)  << fixed << gps.position.altitude + gps.position.geoidalSeparation << endl;

        gps::MB500::display(cout, gps);
    } else {
	size_t count = 0;
	double pos[3] = { 0, 0, 0 };
	while(true)
	{
	    gps.collectPeriodicData();
	    if (gps.position.time == gps.errors.time && (gps.position.time > last_update || last_update == base::Time()))
	    {
		if (gps.position.positionType != NO_SOLUTION && gps.position.positionType != INVALID)
		{
		    if (first_solution.isNull())
		    {
			first_solution = gps.position.time;
			cerr << "first solution found, now waiting " << AVERAGING_TIME << " seconds." << endl;
		    }
		    pos[0] += gps.position.latitude;
		    pos[1] += gps.position.longitude;
		    pos[2] += gps.position.altitude;
		    ++count;
		}

		last_update = gps.position.time;
                gps::MB500::display(cerr, gps) << endl;
	    }

	    if (!first_solution.isNull() && (gps.position.time - first_solution) > base::Time::fromSeconds(AVERAGING_TIME))
	    {
		cerr << "now setting base station position" << endl;
		break;
	    }
	}

	gps.stopPeriodicData();
	pos[0] /= count; pos[1] /= count; pos[2] /= count;
	cerr << "setting fixed position to current position." << endl;
	//cerr << "setting position to: lat=" << pos[0] << ", long=" << pos[1] << ", alt=" << pos[2] << endl;
	//gps.setPosition(pos[0], pos[1], pos[2]);
	gps.setPositionFromCurrent();
    }
    gps.setRTKBase(current_port);
    char buffer[1024];

    last_update = base::Time::now();
    int bytes_tx = 0;
    while(true)
    {
	int rd = read(gps.getFileDescriptor(), buffer, 1024);
        if (rd > 0)
        {
	    int written = 0;
	    while( written < rd )
	    {
		int res = write(diff_io, buffer + written, rd - written);
		if (res == -1)
		{
		    // if ECONNREFUSED, there's nobody at the other end
		    if (errno != ECONNREFUSED && errno != EAGAIN) {
			cerr << "error during write: " << strerror(errno) << endl;
		    	break;
		    }
		}
		else
		    written += res;
	    }

	    bytes_tx += rd;
	    base::Time now = base::Time::now();
	    float duration = (now - last_update).toSeconds();
	    if (duration > 60)
	    {
		cerr << (bytes_tx) << " b/m" << endl;
		bytes_tx = 0;
		last_update = now;
	    }
        }
        usleep(50000);
    }

    return 0;
}

