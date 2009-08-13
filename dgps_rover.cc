#include "dgps.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <iomanip>
#include <fcntl.h>

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

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

int main (int argc, const char** argv){
    DGPS gps;

    if (argc != 4)
    {
        cerr << "usage: dgps_test device_name port_name correction_port" << endl;
        return 1;
    }

    string device_name = argv[1];
    string port_name   = argv[2];

    int correction_socket = openSocket(argv[3]);
    file_guard guard_socket(correction_socket);

    if(!gps.open(device_name))
        return 1;

    if (!gps.setGLONASSTracking(true))
    {
        cerr << "could not enable GLONASS tracking" << endl;
        return 1;
    }
    if (!gps.setFastRTK(false))
    {
        cerr << "could not disable fast RTK" << endl;
        return 1;
    }
    if (!gps.setCodeCorrelatorMode(DGPS::STROBE_CORRELATOR))
    {
        cerr << "could not set code correlator" << endl;
        return 1;
    }
    if (!gps.setRTKInputPort(port_name))
    {
        cerr << "could not setup correction input" << endl;
        return 1;
    }
    gps.setPeriodicData(port_name, 1);
    cout << "DGPS board initialized" << endl;
    char const* fields[12] = {
        "time", "long", "lat", "alt", "dlong", "dlat", "dalt", "sol_type", "sat_count",
        "gps", "sbas", "glonass" };

    for (int i = 0; i < 12; ++i)
        cout << setw(10) << fields[i] << " ";
    cout << endl;

    DFKI::Time last_update;

    char buffer[1024];
    int diff_count = 0;
    while(true)
    {
        fd_set fds;
        FD_ZERO(&fds);
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
        
        if (FD_ISSET(correction_socket, &fds))
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
            gps.collectPeriodicData();
            if (gps.position.timestamp == gps.errors.timestamp && (gps.position.timestamp > last_update || last_update == DFKI::Time()))
            {
                last_update = gps.position.timestamp;
                DGPS::display(cout, gps) << " " << diff_count << endl;
                diff_count = 0;
            }
        }
    }
    gps.close();

    return 0;
}

