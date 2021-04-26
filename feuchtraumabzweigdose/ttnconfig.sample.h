#ifndef TTNCONFIG_H
#define TTNCONFIG_H

// UPDATE WITH YOUR TTN DATA
// If not done yet, please also edit your LMIC project_config and specify your regional LoRAWAN parameters.

// Choose OTAA (true) or ABP (false). OTAA is recommended.
#define USE_OTAA true

// ********************** OTAA **********************
#if USE_OTAA == true

RTC_DATA_ATTR static u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Application EUI (lsb)
RTC_DATA_ATTR static u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Device EUI (lsb)
RTC_DATA_ATTR static u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App Key (msb)

// ********************** ABP **********************
#else

RTC_DATA_ATTR static u1_t PROGMEM NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // LoRaWAN NwkSKey, network session key, hex, msb
RTC_DATA_ATTR static u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // LoRaWAN AppSKey, application session key, hex, msb
RTC_DATA_ATTR static u4_t PROGMEM DEVADDR = 0x00000000 ; // LoRaWAN end-device address (DevAddr), hex, msb

#endif 
// **************************************************

#endif //TTNCONFIG_H