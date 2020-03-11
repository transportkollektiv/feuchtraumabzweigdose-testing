#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

// *****************************************
// DEBUG
// Define this Macro to increase verbosity and update interval

#define DEBUG 1

// *****************************************


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
#define IMU_WAKEUP_FORCE 30

#define RFM_NSS 21
#define RFM_RESET 33
#define RFM_DIO0 26
#define RFM_DIO1 27

// *****************************************
// OTHER PARAMETERS

#define DISTANCE_MOVED 20
#define GPS_TIMEOUT 150000


// *********************************************
// UPDATE WITH YOUR TTN KEYS AND ADDR.


RTC_DATA_ATTR static u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#if TRACKER == 16
#define HAS_IMU MPU9250
RTC_DATA_ATTR static u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
RTC_DATA_ATTR static u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif


#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
