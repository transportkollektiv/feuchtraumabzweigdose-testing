#ifndef CONFIG_H
#define CONFIG_H

// *****************************************
// DEBUG
// Define this Macro to increase verbosity and update interval

// #define DEBUG 1

// *****************************************
// HARDWARE SPECIFICS

// For WROOM:
// DIO0 → IO26
// DIO1 → IO27
// RSET → IO33
// NSS  → IO21
// SCK  → IO18
// MISO → IO19
// MOSI → IO23
//
// MPU9250:
// SCL  → IO22
// SDA  → IO21
// INT  → IO4
// ADD  → GND

#define BUILTIN_LED 14
#define BATTERY_VOLTAGE 32
#define VEXT_ON 25

#define IMU_NCS 13
#define IMU_WAKEUP_FORCE 300

#define RFM_NSS 21
#define RFM_RESET 33
#define RFM_DIO0 26
#define RFM_DIO1 27

#define GPS_TX 17
#define GPS_RX 16

// *****************************************
// OTHER PARAMETERS

#define DISTANCE_MOVED 20
#define GPS_TIMEOUT 150000



#endif //CONFIG_H
