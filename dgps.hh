#ifndef DGPS_H
#define DGPS_H

#include <map>
#include <string>
#include <iosfwd>
#include <sys/types.h>
#include <iodrivers_base.hh>
#include <vector>
#include "dgpstypes.hh"


class DGPS : public IODriver {
	public:
		DGPS();
		~DGPS();

		bool open(const std::string& device_name);
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
                enum DYNAMICS_MODE {
                    STATIC       = 1,
                    QUASI_STATIC = 2,
                    WALKING      = 3,
                    SHIP         = 4,
                    AUTOMOBILE   = 5,
                    AIRCRAFT     = 6,
                    UNLIMITED    = 7,
                    ADAPTIVE     = 8,
                    USER_DEFINED = 9
                };

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

                /** Select the type of receiver motion */
		bool setReceiverDynamics(DYNAMICS_MODE mode);

                /** Forces the receiver to perform PVT initialization on a point
                 * with known geographical coordinates (expressed in the ITRF
                 * model used).
                 */
		bool setKnownPointInit(double, std::string, double, std::string, double, double, double, double, std::string);

                /** Puts the receiver in moving mode, i.e. resets any stored
                 * position */
                bool resetStoredPosition();
		/** Sets the current receiver position. Required to go into RTK base mode. */
		bool setPosition(double latitude, double longitude, double height);
		/** Fixes the receiver position to the currently estimated one */
		bool setPositionFromCurrent();

                /** Enable/disable GLONASS tracking */
		bool setGLONASSTracking(bool);
                /** Enable/disable SBAS tracking */
		bool setSBASTracking(bool);

		enum Special {
			NONE = -1
		};
		bool setCodeMeasurementSmoothing(int, int, int);
		bool setNMEA(std::string, std::string, bool, double = 1);
		bool setNMEALL(std::string, bool);
		bool verifyAcknowledge();

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

		void writeCorrectionData(char const* data, size_t size, int timeout);
	protected:
                float m_period;
		gps::SatelliteInfo tempSatellites;

		static gps::Errors interpretErrors(std::string const& msg);
		static gps::Position interpretInfo(std::string const& msg);
		static bool interpretSatelliteInfo(gps::SatelliteInfo& data, std::string const& msg);
                static DFKI::Time  interpretTime(std::string const& time);

		std::string read(int timeout);
		void write(const std::string&, int timeout);
		int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

        public:
                static std::ostream& display(std::ostream& io, gps::Position const& pos, gps::Errors const& errors, gps::SatelliteInfo const& info);
                static std::ostream& display(std::ostream& io, DGPS const& driver);
};


#endif
