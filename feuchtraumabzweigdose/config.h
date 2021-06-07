#ifndef CONFIG_H
#define CONFIG_H

#include "globals.h"

// ================== GENERAL ==================

// Pinmap
// --------------
// Include the right pinmap that fits your board
//#include "pinmaps/feuchtraumabzw_pcb.h" //default
#include "pinmaps/lock_pcb.h"
//#include "pinmaps/custom.h"

// Debug
// --------------
// Define this Macro to increase verbosity and update interval
// default: disabled

#define DEBUG 1

// Location Source
// --------------
// Activate/Deactivate GPS and Wifi-Scan. Primary sources
// are always used, fallback sources only if the primary 
// source fails (e.g. no GPS fix, no SSIDs found)

// Choose one or disable
#define USE_GPS AS_PRIMARY
//#define USE_GPS AS_FALLBACK

// Choose one or disable
//#define USE_WIFI AS_PRIMARY
#define USE_WIFI AS_FALLBACK


// IMU (Choose one or disable)
// --------------
// An Inertial measurement unit can be connected via 
// the SPI Bus and used as a wakeup signal.
// Currently, only the MPU9250 is supported
//#define HAS_IMU IMU_MPU9250


// Sleeptime
// --------------
// Time the ESP will go to sleep (in seconds). This will 
// approx. equal the time between two measures if no  
// external wakeup occurs. 
// default: 600
#define TIME_TO_SLEEP 600


// Payload (Choose one)
// --------------
// Change this if you want the lora payload to be compatible
// with the Lortinchen Wifitracker 
// (https://github.com/stadtulm/Lora-Wifi-Location-Tracker)
// or with the TTN-Mapper T-Beam GPS Tracker
// (https://github.com/dermatthias/Lora-TTNMapper-T-Beam).
// If you do not want to integrate your device into these
// applications, just use the default payload with the
// provided decoder script.
#define PAYLOAD_TYPE PAYLOAD_DEFAULT
//#define PAYLOAD_TYPE PAYLOAD_WIFITRACKER
//#define PAYLOAD_TYPE PAYLOAD_TBEAM


// ================== GPS Settings ==================

// The following settings are only relevant
// if GPS is enabled in GENERAL

// Minimum supply voltage
// --------------
// To increase battery lifespan, the GPS only
// operates if the supply voltage is above this
// value
// default: 3.3
#define GPS_MIN_VCC 3.3


// GPS fix timeout
// --------------
// Max. time in seconds to wait for GPS fix.
// default: 150
#define GPS_TIMEOUT 150


// GPS fix polling interval
// --------------
// Time between checks for gps fix (in seconds)
// default: 1
#define GPS_POLLING_INTERVAL 5


// Movement detection distance
// --------------
// Minimum distance in meters between two measures
// to be interpreted as movement (otherwise: stationary)
// default: 20
#define DISTANCE_MOVED 20


// Stationary count
// --------------
// When measuring stationary position(s), the device
// will renounce this many times from sending an
// update. Set 0 to always send a location update,
// even if stationary
// default: 1
#define STATCOUNT 1

// ================== Other settings ==================

// IMU Wakeup Threshold
// --------------
// Range: 0 - 1023
// Default: 300
#define IMU_WAKEUP_FORCE 300


#endif //CONFIG_H
