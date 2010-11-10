#ifndef MAGELLAN_MB500_H
#define MAGELLAN_MB500_H

#include <map>
#include <string>
#include <iosfwd>
#include <sys/types.h>
#include <iodrivers_base.hh>
#include <vector>
#include "dgpstypes.hh"

namespace gps {
    /** Driver for the MB500 Magellan differential GPS */
    class MB500 : public IODriver {
    public:
        MB500();
        ~MB500();

        bool openSerial(std::string const& device_name);
        bool open(const std::string& device_name);
        bool openBase(const std::string& device_name);
        bool openRover(const std::string& device_name);

        bool close();

        /** Make the base output RTCM 3.0 correction messages on the provided port */
        bool setRTKBase(std::string port_name);
        /** Stop the output of any RTCM 3.0 messages */
        void stopRTKBase();
        /** Enables or disables Fast-RTK */
        bool setFastRTK(bool enable);
        /** Resets the RTK filter */
        bool setRTKReset(); 
        /** Sets the port on which the RTK corrections will be received
        */
        bool setRTKInputPort(std::string const& port);

        enum CORRELATOR_MODE {
            EDGE_CORRELATOR,
            STROBE_CORRELATOR
        };
        /** Changes the code correlator, for multipath mitigation */
        bool setCodeCorrelatorMode(CORRELATOR_MODE mode);

        /** Reset the board. If \c cold_start is true, reset all stored
         * information about the GNSS constellations
         */
        void reset(bool cold_start);

        /** Dumps the almanac on stdout.
         *
         * Do it right after open()
         */
        void dumpAlmanac();

        /** Dumps the receiver status on stdout
         *
         * Do it right after open()
         */
        void dumpStatus();

        void dumpSatellites();

        /** Select the type of receiver motion */
        bool setReceiverDynamics(gps::DYNAMICS_MODEL mode);

        /** Sets the board's user dynamics parameters. You must call
         * setReceiverDynamics(USER_DEFINED) explicitely afterwards to
         * select this mode. */
        bool setUserDynamics(int h_vel, int h_acc, int v_vec, int v_acc);

        /** Forces the receiver to perform PVT initialization on a point
         * with known geographical coordinates (expressed in the ITRF
         * model used).
         */
        bool setKnownPointInit(double, std::string, double, std::string, double, double, double, double, std::string);

        bool setProcessingRate(int rate);

        /** Puts the receiver in moving mode, i.e. resets any stored
         * position */
        bool resetStoredPosition();
        /** Sets the current receiver position. Required to go into RTK base mode. */
        bool setPosition(double latitude, double longitude, double height);
        /** Fixes the receiver position to the currently estimated one */
        bool setPositionFromCurrent();

        /** Set the ambiguity threshold above which the RTK engine will
         * switch to FIX
         */
        bool setFixThreshold(gps::AMBIGUITY_THRESHOLD threshold);

        void disableAllOutputs();

        /** Enable/disable GLONASS tracking */
        bool setGLONASSTracking(bool);
        /** Enable/disable SBAS tracking */
        bool setSBASTracking(bool);
        /** Sets the acquisition mode */
        bool setGNSSMode(gps::GNSS_MODE mode);

        enum Special {
            NONE = -1
        };
        bool setCodeMeasurementSmoothing(int, int, int);
        bool setNMEA(std::string, std::string, bool, double = 1);
        bool setNMEALL(std::string, bool);
        bool verifyAcknowledge(std::string const& cmd = "");

        std::string getBoardID();

        /** Interprets a NMEA GST message and returns the unmarshalled
         * GST structure
         */
        gps::Errors getGST(std::string msg);
        /** Interprets a NMEA GGA message and returns the unmarshalled
         * GGA structure
         */
        gps::Position getGGA(std::string msg);
        /** Interprets a NMEA GSV message and returns the unmarshalled
         * SatelliteInfo structure
         */
        gps::SatelliteInfo getGSV(std::string msg);

        /** Ask for the GPS receiver to send data periodically. You then
         * call collectPeriodicData() to read the data.
         *
         * @arg period { the update frequency in seconds. Can be one of
         *             0.1, 0.2, 0.5, 1 and any integer greater than 1 }
         *
         * @see collectPeriodicData
         */
        bool setPeriodicData(std::string const& port, double rate);
        /** Reads available data and update the \c data structure. If
         * this method returns true, then \c data has been updated with
         * a new, synchronized set of information. Otherwise, call
         * collectPeriodicData again.
         */
        void collectPeriodicData();
        /** Make the receiver stop sending periodic data */
        bool stopPeriodicData();

        gps::Position position;
        gps::Errors   errors;
        gps::SatelliteInfo satellites;
        gps::SolutionQuality solutionQuality;

        base::Time cpu_time;
        base::Time real_time;
        double processing_latency;

        void writeCorrectionData(char const* data, size_t size, int timeout);

        /** Enable ntpd updates through its shm reference clock driver
         * this needs a line like this in ntp.conf:
         * server 127.127.28.unit
         * where unit can be 0-3, with 0,1 being root-writable and 2,3
         * being world-writable
         *
         * @returns true if the shm area was found
         */
        bool enableNtpdShm(int unit);

    protected:
        float m_period;
        int   m_acq_timeout;

        int ntp_shmid;
        void *ntp_shm;

        // GSV and GSA information are multi-message, so we accumulate
        // information in these temp attributes, and copy them to the
        // real ones whenever the message cycle is finished. 
        gps::SatelliteInfo tempSatellites;
        gps::SolutionQuality tempSolutionQuality;

        bool waitForBoardReset();
        bool interpretQuality(std::string const& message);
        static std::pair<base::Time, base::Time> interpretDateTime(std::string const& msg);
        static gps::Errors interpretErrors(std::string const& msg);
        static gps::Position interpretInfo(std::string const& msg);
        static double interpretLatency(std::string const& message);
        static bool interpretSatelliteInfo(gps::SatelliteInfo& data, std::string const& msg);
        static double interpretAngle(std::string const& value, bool positive);
        static base::Time  interpretTime(std::string const& time);

        void updateNtpdShm();

        std::string read(int timeout);
        void write(const std::string&, int timeout);
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

    public:
        static std::ostream& displayHeader(std::ostream& io);
        static std::ostream& display(std::ostream& io, gps::Position const& pos, gps::Errors const& errors, gps::SatelliteInfo const& info, gps::SolutionQuality const& quality);
        static std::ostream& display(std::ostream& io, MB500 const& driver);
    };
}


#endif
