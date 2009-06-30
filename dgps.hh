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
		int baudrate;

		DGPS();
		~DGPS();
		bool open(const std::string&);
		bool close();
		std::string read(int timeout);
		bool write(const std::string&, int timeout);

		//settings
		enum OnOff {
			ON = true,
			OFF = false
		};
		bool setRTKOutputMode(bool); // RTK Output Mode ON / OFF
		bool setRTKReset(); // reset the RTK processing
		bool setCodeCorrelatorMode(std::string); /* select the type of correlator used for multipath mitigation;
												  * options: "E" - edge correlator , "S" - strobe correlator */
		enum receiverDynamicsOptions {
			STATIC = 1,
			QUASI_STATIC = 2,
			WALKING = 3,
			SHIP = 4,
			AUTOMOBILE = 5,
			AIRCRAFT = 6,
			UNLIMITED = 7,
			ADAPTIVE = 8,
			USER_DEFINED = 9
			};
		bool setReceiverDynamics(int); // select the receiver motion

		bool setKnownPointInit(double, std::string, double, std::string, double, double, double, double, std::string);
		//forces the receiver to perform PVT initialization on a point with known geographical coordinates (expressed in the ITRF model used).

		bool setSBASTracking(bool); // enables/disable the SBAS tracking

		enum Special {
			NONE = -1
		};
		bool setCodeMeasurementSmoothing(int, int, int); // sets the smoothing interval in code measurement
		bool setNMEA(std::string, std::string, bool, double = 1);
		bool setNMEALL(std::string, bool);
		bool verifyAcknowledge();
		bool setPeriodicData();

		//queries
		gps::Errors getGST(std::string = "");
		gps::Info getGGA(std::string = "");
		gps::SatelliteInfo getGSV(std::string = "");

		gps::Errors interpretErrors(std::string &);
		gps::Info interpretInfo(std::string &);
		gps::SatelliteInfo interpretSatelliteInfo(std::string &);
		void collectPeriodicData();
		bool stopPeriodicData();

		gps::FullInfo data;
		gps::SatelliteInfo dataSatellite;
		bool releaseOK;
		bool isOKtoReleaseData() { return releaseOK; };

	protected:
		int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
};

#endif
