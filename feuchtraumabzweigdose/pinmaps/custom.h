#ifndef PINMAPS_CUSTOM_H
#define PINMAPS_CUSTOM_H

// Custom Pinmap
// Use this to define pins if you are using your own, custom board

// WROOM ESP32:
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
#define IMU_WAKE 4

#define RFM_NSS 21
#define RFM_RESET 33
#define RFM_DIO0 26
#define RFM_DIO1 27

#define GPS_TX 17
#define GPS_RX 16

#endif