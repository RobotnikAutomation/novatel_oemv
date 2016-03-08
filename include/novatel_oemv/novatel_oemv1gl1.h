/** \file novatel_oemv1gl1.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief class for the Novatel OEMV 1GL1 RTK-DGPS rover
 * (C) 2012 Robotnik Automation, SLL
*/

//#include <pthread.h>
//#include "Component.h"
//#include "SerialDevice.h"
#include "novatel_oemv/Component.h"
#include "novatel_oemv/SerialDevice.h"
#include <stdint.h>
#include <string>

#include <iostream>
#include <fstream>

using namespace std;
using std::ofstream;

#ifndef __NOVATEL_OEMV1GL1_H
	#define __NOVATEL_OEMV1GL1_H

#define BUFFER_SIZE					  1000


// ERROR FLAGS
#define NOVATEL_OEMV1GL1_ERROR_NONE				 0
#define NOVATEL_OEMV1GL1_ERROR_OPENING			 1
#define NOVATEL_OEMV1GL1_ERROR_SERIALCOMM		 2
#define NOVATEL_OEMV1GL1_ERROR_TIMEOUT			 3

#define NOVATEL_OEMV1GL1_OK						 0
#define NOVATEL_OEMV1GL1_ERROR					-1

//! Timeout for controlling the communication with the device (in seconds)
#define NOVATEL_OEMV1GL1_TIMEOUT_COMM		     5.0

//! Desired frequency
#define NOVATEL_OEMV1GL1_THREAD_DESIRED_HZ		 100    // Frequency also used by default in rtklib
#define NOVATEL_OEMV1GL1_DEFAULT_SERIAL_HZ       100
#define RUNNING							         3
#define NOVATEL_OEMV1GL1_DEFAULT_DATA_SIZE		    8
#define NOVATEL_OEMV1GL1_DEFAULT_PORT				"/dev/ttyUSB0"
#define NOVATEL_OEMV1GL1_DEFAULT_TRANSFERRATE	 	    9600  //19200
#define NOVATEL_OEMV1GL1_DEFAULT_PARITY 			"none" 	//"even" "odd""none"

#define NOVATEL_CRC32_POLYNOMIAL    0xEDB88320L

//! Frame types
typedef struct {
    double x;
    double y;
    double z;
} DPoint;

//! Frame types
typedef struct {
    float x;
    float y;
    float z;
} FPoint;

//! Gps time status type
enum gps_time_status_t { _UNKNOWN=20, _APPROXIMATE=60, _COARSEADJUSTING=80, _COARSE=100,
                         _COARSESTEERING=120, _FREEWHEELING=130, _FINEADJUSTING=140, _FINE=160,
                         _FINESTEERING=180, _SATTIME=200 };

/*
20  UNKNOWN         Time validity is unknown.
60  APPROXIMATE     Time is set approximately.
80  COARSEADJUSTING Time is approaching coarse precision.
100 COARSE          This time is valid to coarse precision.
120 COARSESTEERING  Time is coarse set, and is being steered.
130 FREEWHEELING    Position is lost, and the range bias cannot be calculated.
140 FINEADJUSTING   Time is adjusting to fine precision.
160 FINE            Time has fine precision.
180 FINESTEERING    Time is fine, set and is being steered.
200 SATTIME         Time from satellite. This is only used in logs containing
                    satellite data such as ephemeris and almanac.
*/

//! ASCII Message Header Structure
typedef struct {
        char cmd[20];   // Sync Char Sync character. The ASCII message is always preceded by a single ‘#’ symbol.
                        // Message Char  This is the ASCII name of the log or command
        char cmdPort[10]; // This is the name of the port from which the log was
                        // generated. The string is made up of the port name
                        // followed by an _x where x is a number from 1 to 31
                        // denoting the virtual address of the port. If no virtual
                        // address is indicated, it is assumed to be address 0.
        long sequence;  // This is used for multiple related logs. It is a number
                        // that counts down from N-1 to 0 where 0 means it is
                        // the last one of the set. Most logs only come out one
                        // at a time in which case this number is 0.
        float percent_idle_time; // Idle Time Float The minimum percentage of time that the processor
                        // is idle between successive logs with the same
                        // Message ID.
        enum gps_time_status_t gps_time_status;   // This value indicates the quality of the GPS time
        unsigned long gps_week_number;            //  GPS week number
        float seconds;  // GPSec Seconds from the beginning of the GPS week
                        // accurate to the millisecond level.
                        // ! In binary frame long, in ascii frame float
        unsigned long receiver_status; // This is an eight digit hexadecimal number
                        // representing the status of various hardware and
                        // software components of the receiver between
                        // successive logs with the same Message ID
        unsigned long reserved; // Reserved for internal use.
        unsigned long receiver_sw_version;  // This is a value (0 - 65535) that represents the
                        // receiver software build number.
} ascii_msg_header_t;

//! Position or velocity solution status type
enum pv_sol_status_t { _SOL_COMPUTED = 0, _INSUFFICIENT_OBS = 1, _NO_CONVERGENCE = 2, _SINGULARITY = 3, _COV_TRACE = 4,
        _TEST_DIST = 5, _COLD_START = 6, _V_H_LIMIT = 7, _VARIANCE = 8, _RESIDUALS = 9, _DELTA_POS  = 10, _NEGATIVE_VAR = 11,
        _RESERVED = 12, _INTEGRITY_WARNING = 13, _INS1 = 14, _INS2 = 15, _INS3 = 16, _INS4 = 17, _PENDING = 18, _INVALID_FIX = 19,
        _UNAUTHORIZED = 20, _ANTENNA_WARNING = 21
        };

/*
0   SOL_COMPUTED        Solution computed
1   _INSUFFICIENT_OBS   Insufficient observations
2   _NO_CONVERGENCE     No convergence
3   _SINGULARITY        Singularity at parameters matrix
4   _COV_TRACE          Covariance trace exceeds maximum (trace > 1000 m)
5   _TEST_DIST          Test distance exceeded (maximum of 3 rejections if distance > 10 km)
6   _COLD_START         Not yet converged from cold start
7   _V_H_LIMIT          Height or velocity limits exceeded (in accordance with export licensing restrictions)
8   _VARIANCE           Variance exceeds limits
9   _RESIDUALS          Residuals are too large
10  _DELTA_POS          Delta position is too large
11  _NEGATIVE_VAR       Negative variance
12  _RESERVED
13  _INTEGRITY_WARNING  Large residuals make position unreliable
14-17  _INS1,2,3,4      Solution status values 1 (only in combination with novatel inertial prods)
18  _PENDING            When a FIX POSITION command is entered, the
                        receiver computes its own position and determines if
                        the fixed position is valid b
19  _INVALID_FIX        The fixed position, entered using the FIX POSITION
                        command, is not valid
20  _UNAUTHORIZED       Position type is unauthorized - HP or XP on a receiver
                        not authorized for it
21 _ANTENNA_WARNING     One of the antenna warnings listed in the
                        RTKANTENNA command description
*/

//! Position or velocity type
enum pv_type_t { _NONE=0, _FIXEDPOS = 1, _FIXEDHEIGHT = 2, _DOPPLER_VELOCITY=8, _SINGLE = 16, _PSRDIFF = 17, _WAAS = 18, _PROPAGATED = 19,
                  _OMNISTAR = 20, _L1_FLOAT=32, _IONOFREE_FLOAT = 33, _NARROW_FLOAT = 34 , _L1_INT=48, _WIDE_INT = 49, _NARROW_INT = 50,
                  _RTK_DIRECT_INS = 51, _INS = 52, _INS_PSRSP = 53,
                  _INS_PSRDIFF = 54, _INS_RTKFLOAT = 55, _INS_RTKFIXED = 56, _OMNISTAR_HP=64, _OMNISTAR_XP = 65, _CDGPS = 66};

/*
0  NONE             No solution
1  FIXEDPOS         Position has been fixed by the FIX POSITION command
2  FIXEDHEIGHT      Position has been fixed by the FIX HEIGHT/AUTO command
8  DOPPLER_VELOCITY Velocity computed using instantaneous Doppler
16 SINGLE           Single point position
17 PSRDIFF          Pseudorange differential solution
18 WAAS             Solution calculated using corrections from a SBAS
19 PROPAGATED       Propagated by a Kalman filter without new observations
20 OMNISTAR         OmniSTAR VBS position (L1 sub-meter)
32 L1_FLOAT         Floating L1 ambiguity solution
33 IONOFREE_FLOAT   Floating ionospheric-free ambiguity solution
34 NARROW_FLOAT     Floating narrow-lane ambiguity solution
48 L1_INT           Integer L1 ambiguity solution
49 WIDE_INT         Integer wide-lane ambiguity solution
50 NARROW_INT       Integer narrow-lane ambiguity solution RTK status where the RTK filter is directly initialized from
51 RTK_DIRECT_INS   RTK status where the RTK filter is directly initialized from the INS filter
52 INS              INS calculated position corrected for the antenna
53 INS_PSRSP        INS pseudorange single point solution - no DGPS corrections
54 INS_PSRDIFF      INS pseudorange differential solution
55 INS_RTKFLOAT     INS RTK floating point ambiguities solution
56 INS_RTKFIXED     INS RTK fixed ambiguities solution
64 OMNISTAR_HP      OmniSTAR HP position
65 OMNISTAR_XP      OmniSTAR XP position
66 CDGPS            Position solution using CDGPS correction
*/

//! BESTXYZA frame type
typedef struct {
   ascii_msg_header_t   amh;    // ascii message header
   enum pv_sol_status_t  p_sol_status;  // position solution status
   enum pv_type_t pos_type;             // position type
   DPoint p;            // point x y z
   FPoint p_std;        // standard deviation x y z
   enum pv_sol_status_t  v_sol_status;  // velocity solution status
   enum pv_type_t vel_type;             // velocity type
   DPoint v;            // velocity x y z
   FPoint v_std;        // standard deviation x y z
   char stnID[4];       // base station identification
   float v_latency;     // latency in the velocity time tag [s]
   float diff_age;      // differential age in [s]
   float sol_age;       // solution age in [s]
   unsigned char SVs;   // number of satellite vehicles tracked
   unsigned char solnSVs;   // number of satellite vehicles used in solution
   unsigned char ggL1;      // number of gps plus glonass L1 used in solution
   unsigned char ggL1L2;    // number of gps plus glonass L1 and L2 used in solution
   char reserved1;
   unsigned char ext_sol_stat;  // (hex) extended solution status
   char reserved2;
   unsigned char sig_mask;  // (hex) signals used mask 0=unknown
   int32_t crc;         // 32 bit crc
   bool bReceived;      // true if one frame of this type has been received
} bestxyza_t;

//! PDPXYZA frame type
typedef struct {
   ascii_msg_header_t   amh;    // ascii message header
   enum pv_sol_status_t  p_sol_status;  // position solution status
   enum pv_type_t pos_type;             // position type
   DPoint p;            // point x y z
   FPoint p_std;        // standard deviation x y z
   enum pv_sol_status_t  v_sol_status;  // velocity solution status
   enum pv_type_t vel_type;             // velocity type
   DPoint v;            // velocity x y z
   FPoint v_std;        // standard deviation x y z
   char stnID[4];       // base station identification
   float v_latency;     // latency in the velocity time tag [s]
   float diff_age;      // differential age in [s]
   float sol_age;       // solution age in [s]
   unsigned char SVs;   // number of satellite vehicles tracked
   unsigned char solnSVs;   // number of satellite vehicles used in solution
   char reserved[6];    // 
   int32_t crc;         // 32 bit crc
   bool bReceived;      // true if one frame of this type has been received
} pdpxyza_t;

//! BESTPOSA frame type
typedef struct {
   ascii_msg_header_t   amh;    // ascii message header
   enum pv_sol_status_t  p_sol_status;  // position solution status
   enum pv_type_t pos_type;             // position type
   double lat;                  // latitude
   double lon;                  // longitude
   double hgt;                  // height above mean sea level
   float undulation;            // Undulation - the relationship between the geoid and
                                // the ellipsoid (m) of the chosen datum a
   //enum datum_id_t datum_id;  // Datum ID number (see Chapter 2, Table 21,
   float lat_std;               // Latitude standard deviation
   float lon_std;               // Longitude standard deviation
   float hgt_std;               // Height standard deviation
   char stnID[4];               // Base station ID
   float diff_age;      	// differential age in [s]
   float sol_age;       	// solution age in [s]
   unsigned char SVs;   	// number of satellite vehicles tracked
   unsigned char solnSVs;   // number of satellite vehicles used in solution
   unsigned char ggL1;      // number of gps plus glonass L1 used in solution
   unsigned char ggL1L2;    // number of gps plus glonass L1 and L2 used in solution
   char reserved1;
   unsigned char ext_sol_stat;  // (hex) extended solution status
   char reserved2;
   unsigned char sig_mask;  // (hex) signals used mask 0=unknown
   int32_t crc;         // 32 bit crc
   bool bReceived;      // true if one frame of this type has been received
} bestposa_t;

//! PDPPOSA frame type
typedef struct {
   ascii_msg_header_t   amh;    // ascii message header
   enum pv_sol_status_t  p_sol_status;  // position solution status
   enum pv_type_t pos_type;             // position type
   double lat;                  // latitude
   double lon;                  // longitude
   double hgt;                  // height above mean sea level
   float undulation;            // Undulation - the relationship between the geoid and
                                // the ellipsoid (m) of the chosen datum a
   //enum datum_id_t datum_id;  // Datum ID number (see Chapter 2, Table 21,
   float lat_std;               // Latitude standard deviation
   float lon_std;               // Longitude standard deviation
   float hgt_std;               // Height standard deviation
   char stnID[4];               // Base station ID
   float diff_age;      	// differential age in [s]
   float sol_age;       	// solution age in [s]
   unsigned char SVs;   	// number of satellite vehicles tracked
   unsigned char solnSVs;   // number of satellite vehicles used in solution
   char reserved[6];    	// 
   int32_t crc;         	// 32 bit crc
   bool bReceived;      	// true if one frame of this type has been received
} pdpposa_t;


//! IONUCTA frame type
typedef struct {
    ascii_msg_header_t   amh; // ascii message header
    double a0;          // Alpha parameter constant term
    double a1;          // Alpha parameter 1st order term
    double a2;          // Alpha parameter 2nd order term
    double a3;          // Alpha parameter 3rd order term
    double b0;          // Beta parameter constant term
    double b1;          // Beta parameter 1st order term
    double b2;          // Beta parameter 2nd order term
    double b3;          // Beta parameter 3rd order term
    unsigned long utc_wn;   // UTC reference week number
    unsigned long tot;  // Reference time of UTC parameters
    double A0;          // UTC constant term of polynomial
    double A1;          // UTC 1st order term of polynomial
    unsigned long wn_lsf;   // Future week number
    unsigned long dn;   // Day number (the range is 1 to 7 where
                        // Sunday = 1 and Saturday = 7)
    long deltat_ls;     // Delta time due to leap seconds
    long deltat_lsf;    // Future delta time due to leap seconds
    unsigned long deltat_utc; // Time difference
    int32_t crc;        // 32-bit CRC (ASCII and Binary only)
    bool bReceived;      // true if one frame of this type has been received
} ionutca_t;

//! VERSIONA frame type
typedef struct {
    ascii_msg_header_t   amh;   // ascii message header
    long comp;                  // Number of componentes
    char type[20];              // OEMV family component type: GPSCARD, CONTROLLER, ENCLOSURE, IMUCARD
    char model[16];             // A base model name plus designators where
                                // there are 4 possible base names:
                                // L12:     20 Hz pos and measurements, RT2/20 base, 14 GPS L1/L2 and 2 SBAS channels
                                // L1:      20 Hz pos and measurements,RT20 base, 14 GPS L1 and 2 SBAS channels
                                // N12:     20 Hz positions, no measurements, GPS L1/L2 and 2 SBAS channels
                                // N1:      20 Hz positions, no measurements, GPS L1 and 2 SBAS channels
				// D1SB0G55N ?
    char psn[16];               // Product serial number
    char hw_version[16];        // Hardware version
//    char sw_version[16];        // Firmware software version
    char sw_version[20];        // Firmware software version (upgrade to OEMV6)
//    char boot_version[16];      // Boot code version
    char boot_version[20];      // Boot code version (upgrade to OEMV6)
//    char comp_date[12];         // Firmware compile date
    char comp_date[16];         // Firmware compile date (upgrade to OEMV6)
    char comp_time[12];         // Firmware compile time
    int32_t crc;                // 32-bit CRC (ASCII and Binary only)
    bool bReceived;      // true if one frame of this type has been received
} versiona_t;

//! Class to operate with the novatel oemv1gl1 DGPS/RTK sensor
class novatel_oemv1gl1 : public Component {

public:
	//! Buffer for receiving data from the device
	char RecBuf[BUFFER_SIZE];

private:

	//! Mutex for controlling the changes and access to odomety values
	pthread_mutex_t mutex_record;
	//! Contains serial device structure for port
	SerialDevice *serial;
	//! Status of novatel_oemv1gl1
	int iStatusnovatel_oemv1gl1;
	//! Auxiliar variable for controling the timeout in the communication
	struct timespec tNext;
	//! Auxiliar variable for controling the timeout in the communication
	struct timespec	tNow;
	//! Contains the last ocurred error
	int iErrorType;
	//! Contains data of the bestxyza frame
	bestxyza_t bestxyza;
	//! Contains data of the pdpxyza frame
	pdpxyza_t pdpxyza;
	//! Contains data of the bestposa frame
	bestposa_t bestposa;
	//! Contains data of the ionutca frame	
	ionutca_t ionutca;
	//! Contains data of the pdpposa frame
	pdpposa_t pdpposa;
	//! Contains data of the versiona frame
	versiona_t versiona;
    //! Kml output file stream
    ofstream kml_ofstream;
    //! Csv output file streams
    ofstream csv_ofstream;
    ofstream pdpcsv_ofstream;
    //! Full log file stream
    ofstream full_ofstream;
    //! String to determine mode
    string mode_;   

public:

	//! Public constructor
	//novatel_oemv1gl1(const char *channel, int baud_rate, const char *parity, int datasize, double hz);
	// novatel_oemv1gl1( double hz );
	novatel_oemv1gl1( const char *device, double hz, string mode );
	//! Public destructor
	~novatel_oemv1gl1();
	//!	calculates checksum of received frame
	int16_t Checksum(const char* pBytes, int count);
    //! Display variables for testing component
    void Display_message();
    //! Returns data structure with gps data structure of player
//    player_gps_data_t GetPlayerGpsPacket();
    //! Sends reset to factory defaults
    void FReset();
    //! Gets the current bestxyza value
    bestxyza_t GetBestxyza();
    //! Gets the current bestposa value
    bestposa_t GetBestposa();
    //! Gets the current pdpxyza value
    pdpxyza_t GetPdpxyza();
    //! Gets the current Ionutca value
    ionutca_t GetIonutca();    
    //! Gets the current pdpposa value
    pdpposa_t GetPdpposa();
    //! Sends the commands to stop logging
	int SendUnlogCommands();
	//! Enables/Disables the configuration to use EGNOS
	void SetEgnos(bool value);
	//! Enables/Disables the configuration to use the PDP filter
	void SetPDPFilter(bool value);

private:

	//! Open serial ports 1 and 2
	ReturnValue Open();
	//! Closes serial port
	ReturnValue Close();
    //! Sends command to serial device
	int Send(int command, int serial);
	//! Reads data received from device
	int Read();
    //! Sends strings to the SerialDevice
    ReturnValue SendMessage(char *msg);
	//! Actions in initial state
	void InitState();
    //! Sends the configuration commands to start using the RTK DGPS
    int SendInitCommands();
	//! Actions in ready state
	void ReadyState();
	//! Actions in emergency state
	void EmergencyState();
    //! Read and process OEMV4 type frames from RTK DGPS
    int ReadOEMV4Frame();
    //! Process OEMV4 type frames from RTK DGPS
    int ProcessOEMV4Frame(char* str);
    //! Process ASCII message header
    int ProcessAsciiMsgHeader(char* str, ascii_msg_header_t* _amh);
    //! Process #BESTXYZA frame
    int ProcessFrameBestxyza(char* str);
    //! Process #PDPXYZA frame
    int ProcessFramePdpxyza(char* str);
    //! Process #BESTPOSA frame
    int ProcessFrameBestposa(char* str);
    //! Process #PDPPOSA frame
    int ProcessFramePdpposa(char* str);
    //! Process #IONUTCA frame
    int ProcessFrameIonutca(char* str);
    //! Process #VERSIONA frame
    int ProcessFrameVersiona(char* str);
    //! Calculate a CRC value to be used by CRC calculation functions
    unsigned long CRC32Value(int i);
    //! Calculates the CRC-32 of a block of data all at once
    unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer );
    //! Open all log files
    void OpenLogFiles();
    //! Open KML file to start writing tracks and write header
    void OpenKMLFile();
    //! Write end and close KML file
    void CloseKMLFile();
    //! Open CSV file
    void OpenCSVFile();
    //! Close CSV file
    void CloseCSVFile();
    //! Close log files
    void CloseLogFiles();
    //! Processes the GPS' status string from the device and converts it to an integer value
    enum gps_time_status_t StringToGpsStatus(char *val);
    //! Processes the type string from the device and converts it to an integer value
    enum pv_type_t StringToType(char *val);
    //! Processes the Solution status string from the device and converts it to an integer value
    enum pv_sol_status_t StringToSolStatus(char *val);
};

#endif
