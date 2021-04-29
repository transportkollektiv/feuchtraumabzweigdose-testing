#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <driver/rtc_io.h>

#include "config.h"

#if __has_include("ttnconfig.h")
#include "ttnconfig.h"
#else
#error "No ttnconfig.h file found. Please rename ttnconfig.sample.h to ttnconfig.h and fill in your TTN Keys!"
#endif

extern RTC_DATA_ATTR u4_t RTC_seqnoUp;
extern RTC_DATA_ATTR int otaaJoined;
extern RTC_DATA_ATTR u1_t keep_nwkKey[16];
extern RTC_DATA_ATTR u1_t keep_artKey[16];
extern RTC_DATA_ATTR devaddr_t keep_devaddr;
extern osjob_t sendjob;
extern const lmic_pinmap lmic_pins;
extern char s[32];

extern uint8_t txBuffer[11];

void os_getArtEui(u1_t *buf);
void os_getDevEui(u1_t *buf);
void os_getDevKey(u1_t *buf);
void do_send(osjob_t *j);
void lora_init(int bootCount);
void lora_onEvent(ev_t ev);
void lora_send ();



#endif //LORA_H