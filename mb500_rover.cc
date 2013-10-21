#include "mb500.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>
#include <fcntl.h>

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

#include <memory>

using namespace std;

int openSocket(std::string const& port)
{
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
    hints.ai_protocol = 0;          /* Any protocol */
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;

    struct addrinfo *result;
    int s = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (s != 0) {
        fprintf(stderr, "cannot bind to port %s: %s\n", port.c_str(), gai_strerror(s));
        return -1;
    }

    int sfd = -1;
    struct addrinfo *rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype,
                rp->ai_protocol);
        if (sfd == -1)
            continue;

        if (bind(sfd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;                  /* Success */

        close(sfd);
    }

    freeaddrinfo(result);           /* No longer needed */

    if (rp == NULL)
    {
        fprintf(stderr, "Could not bind\n");
        return -1;
    }

    // Set the socket as nonblocking
    long fd_flags = fcntl(sfd, F_GETFL);                                                                                             
    fcntl(sfd, F_SETFL, fd_flags | O_NONBLOCK);                                                                                      

    return sfd;
}

void usage()
{
    cerr << "usage: dgps_rover device_name port_name correction_source" << endl;
    cerr << "  where correction_source is either a port number or a\n"
        "  Magellan port name. In the first case, corrections are\n"
        "  expected as UDP packets sent to the given port, and\n"
        "  are sent to the command port port_name. In the second\n"
        "  case, corrections are expected on the given board port,\n"
        "  which has to be different than port_name" << endl;
}

int main (int argc, const char** argv){
    gps::MB500 gps;

    if (argc != 4)
    {
        usage();
        return 1;
    }

    string device_name = argv[1];
    string port_name   = argv[2];
    string correction_source = argv[3];

    int correction_socket = -1;
    string correction_input_port;
    std::auto_ptr<iodrivers_base::FileGuard> guard_socket;

    if (correction_source.size() != 1 || correction_source.find_first_of("ABC") != 0)
    {
        cerr << "reading correction data from UDP port " << correction_source << endl;
        correction_socket = openSocket(correction_source);
        guard_socket.reset(new iodrivers_base::FileGuard(correction_socket));
        correction_input_port = port_name;
    }
    else
    {
        cerr << "correction data will be sent to the board's port " << correction_source << endl;
        if (port_name == correction_source)
        {
            std::cerr << "you cannot use the same port for port_name and correction_source" << std::endl;
            usage();
            return 1;
        }

        correction_input_port = correction_source;
    }

    if(!gps.openRover(device_name))
        return 1;

    if (!gps.setFastRTK(false))
    {
        cerr << "could not disable fast RTK" << endl;
        return 1;
    }
    if (!gps.setRTKInputPort(correction_input_port))
    {
        cerr << "could not setup correction input" << endl;
        return 1;
    }
    gps.setPeriodicData(port_name, 1);
    cout << "gps::MB500 board initialized" << endl;
    gps::MB500::displayHeader(cout);

    base::Time last_update;

    char buffer[1024];
    int diff_count = 0;
    int seq = 0;
    while(true)
    {
        fd_set fds;
        FD_ZERO(&fds);
        if (correction_socket != -1)
            FD_SET(correction_socket, &fds);
        FD_SET(gps.getFileDescriptor(), &fds);
        int ret = select(std::max(correction_socket, gps.getFileDescriptor()) + 1, &fds, NULL, NULL, NULL);
        if (ret < 0)
        {
            cerr << "error during select()" << endl;
            return 1;
        }
        else if (ret == 0)
            cerr << "zero return value" << endl;
        
        if (correction_socket != -1 && FD_ISSET(correction_socket, &fds))
        {
            int rd = recv(correction_socket, buffer, 1024, 0);
            if (rd > 0)
            {
                gps.writeCorrectionData(buffer, rd, 1000);
                diff_count += rd;
            }
            else if (rd < 0)
            {
                cerr << "error reading socket: " << strerror(errno) << endl;
            }
        }

        if (FD_ISSET(gps.getFileDescriptor(), &fds))
        {
            try {
                gps.collectPeriodicData();
                if (gps.position.time == gps.errors.time && (gps.position.time > last_update || last_update == base::Time()))
                {
                    ++seq;
                    last_update = gps.position.time;
                    cout << seq << " ";
                    gps::MB500::display(cout, gps) << " " << diff_count << endl;
                    diff_count = 0;
                }
            }
            catch(iodrivers_base::TimeoutError) {}
        }
    }
    gps.close();

    return 0;
}

