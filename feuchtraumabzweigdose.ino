#define FW_VERSION "20191023-0.4.1"
#define TRACKER 16

#include <lmic.h>
#include <hal/hal.h>
#include <esp_wifi.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "gps.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#ifdef DEBUG
  #define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */
  #define STATCOUNT 1
#else
  #ifdef HAS_IMU
    #define TIME_TO_SLEEP 600
  #else
    #define TIME_TO_SLEEP 180
  #endif
  #define STATCOUNT 2
#endif

#if defined(HAS_IMU) && HAS_IMU == MPU9250
  // MPU Accelerometer for wake on motion
  #include "MPU9250.h"
  MPU9250 IMU(SPI,IMU_NCS);
  int status;
#endif

// OTAA (true) or ABP (false)
#define USE_OTAA true

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR u4_t RTC_seqnoUp = 0;
RTC_DATA_ATTR int otaaJoined = 0;
RTC_DATA_ATTR u1_t keep_nwkKey[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
RTC_DATA_ATTR u1_t keep_artKey[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
RTC_DATA_ATTR devaddr_t keep_devaddr = 0;

RTC_DATA_ATTR double prevLat = 0;
RTC_DATA_ATTR double prevLon = 0;
RTC_DATA_ATTR int statCount = 0;
double dist = 0;
bool hasFix = false;

esp_sleep_wakeup_cause_t wakeup_reason;

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[11];
gps gps;

// OTAA, see config.h for settings
#if defined(USE_OTAA) && USE_OTAA == true
void os_getArtEui(u1_t * buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t * buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t * buf) { memcpy_P(buf, APPKEY, 16); }
#else
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t * buf) {}
void os_getDevEui(u1_t * buf) {}
void os_getDevKey(u1_t * buf) {}
#endif

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = RFM_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RFM_RESET,
  .dio = {RFM_DIO0, RFM_DIO1, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      Serial.println(F("Saving OTAA values after successfull join."));
      memcpy(keep_nwkKey, LMIC.nwkKey, 16);
      memcpy(keep_artKey, LMIC.artKey, 16);
      keep_devaddr = LMIC.devaddr;

      for(int i = 0; i < 16; i++) {
        Serial.print("0x");
        Serial.print(LMIC.nwkKey[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");

      otaaJoined = 1;
      LMIC_setLinkCheckMode(0);
      break;
    //case EV_RFU1:
    //  Serial.println(F("EV_RFU1"));
    //  break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      // go into deep sleep for TX_interval
      RTC_seqnoUp = LMIC.seqnoUp;
      lowPower();
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      Serial.println((unsigned) ev);
      break;
  }
}

float getBatteryVoltage() {
  // first read of voltage; this is the least accurate and will be discarded.
  int current_reading = analogRead(BATTERY_VOLTAGE);
  int average_reading = 0;
  // read voltage multiple times and average over all readings
  for (int i = 0; i < 5; i++) {
    current_reading = analogRead(BATTERY_VOLTAGE);
    average_reading += current_reading;
  }
  //float vBat = average_reading / 1024 * 3.3 * 100.0;
  //float vBat = (3.3 / 4096.0) * ( average_reading / 5.0) * 2.268;
    float vBat = (3.3 / 4096.0) * ( average_reading / 5.0) * 226.8;

  return vBat;
}

void do_send(osjob_t* j) {  
  
  uint16_t currentVoltage = getBatteryVoltage() * 100;
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}



void setup() {

  //Turn off WiFi and Bluetooth
  //WiFi.mode(WIFI_OFF);
  //esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  btStop();
  
  // Setup ADC to measure battery voltage
  //adcAttachPin(BATTERY_VOLTAGE);
  //adcStart(BATTERY_VOLTAGE);
  //analogReadResolution(10);
  uint16_t startVoltage = getBatteryVoltage();

  txBuffer[9] = (startVoltage >> 8);
  txBuffer[10] = startVoltage;
  
  // setup Vext pin, keep low until LoRa is setup and it is clear that voltage is sufficient
  pinMode(VEXT_ON, OUTPUT);
  digitalWrite(VEXT_ON, LOW);
  
  Serial.begin(115200);
  delay(100);
  Serial.println(FW_VERSION);

  wakeup_reason = esp_sleep_get_wakeup_cause();

  //Increment boot number and print it every reboot
  ++bootCount;
  #ifdef DEBUG
    Serial.println("Boot number: " + String(bootCount));
    Serial.println("RTC_seqnoUp: " + String(RTC_seqnoUp));
    Serial.println("Stationary Counter: " + String(statCount));    
    Serial.println("Battery Voltage: " + String(startVoltage)); 
    Serial.print("Real Battery Voltage from ADC: ");
    Serial.println(analogRead(BATTERY_VOLTAGE));
  
    //Print the wakeup reason for ESP32
    switch (wakeup_reason)
      {
        case ESP_SLEEP_WAKEUP_EXT0  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
        case ESP_SLEEP_WAKEUP_EXT1  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
        case ESP_SLEEP_WAKEUP_TIMER  : Serial.println("Wakeup caused by timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD  : Serial.println("Wakeup caused by touchpad"); break;
        case ESP_SLEEP_WAKEUP_ULP  : Serial.println("Wakeup caused by ULP program"); break;
        default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
      } 
  #endif


  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // ABP or OTAA
  #if defined(USE_OTAA) && USE_OTAA == true
    #ifdef DEBUG
      Serial.println("Activation method: OTAA");
    #endif
    if (bootCount > 0 && otaaJoined == 1) {
        Serial.println(F("Restoring OTAA session credentials from memory"));
        memcpy(LMIC.nwkKey, keep_nwkKey, 16);
        memcpy(LMIC.artKey, keep_artKey, 16);
        LMIC.devaddr = keep_devaddr;
      }
  #else
    #ifdef DEBUG
      Serial.println("Activation method: ABP");
    #endif
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9,14); 

  LMIC.seqnoUp = RTC_seqnoUp;
  LMIC.seqnoDn = RTC_seqnoUp;

  //TODO determine whether to send emergency message because of voltage

  gps.init();
  //gps.softwareReset();

  #if defined(HAS_IMU) && HAS_IMU == MPU9250
  // start communication with IMU 
  status = IMU.begin();
  #ifdef DEBUG
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }
  #endif  
  int imo_wom = IMU.enableWakeOnMotion(IMU_WAKEUP_FORCE,MPU9250::LP_ACCEL_ODR_15_63HZ);
  Serial.print("Wake on motion set: ");
  Serial.println(imo_wom);
  #endif

  unsigned long startGpsTime = millis();
  while (millis() < startGpsTime + GPS_TIMEOUT)
    {
      if(gps.checkGpsFix()) {
        digitalWrite(VEXT_ON, LOW);
        dist = TinyGPSPlus::distanceBetween(gps.lat(), gps.lng(), prevLat, prevLon);
     
        if (dist > DISTANCE_MOVED || statCount > STATCOUNT) {
          Serial.println("Distance moved: " + String(dist));
          Serial.println("Time stationary: " + String(statCount * TIME_TO_SLEEP * uS_TO_S_FACTOR));
          if (dist <= DISTANCE_MOVED) {
            Serial.println("Sending because stationary for longer than max.");
           }
          statCount = 0;
          prevLat = gps.lat();
          prevLon = gps.lng();
          // Prepare upstream data transmission at the next possible time.
          gps.buildPacket(txBuffer);
          hasFix = true;
          digitalWrite(VEXT_ON, LOW);
          do_send(&sendjob);
        } else {
          Serial.println("Not sending, stationary.");
          Serial.println("Distance moved: " + String(dist));
          #ifndef HAS_IMU
            Serial.println("Time stationary: " + String(statCount * TIME_TO_SLEEP * uS_TO_S_FACTOR));
          #else
            Serial.println("Time stationary: " + String(statCount));
         #endif
          ++statCount;
          RTC_seqnoUp = LMIC.seqnoUp;
          lowPower();
        }
      break;
      } 
    } 

if (!hasFix) {
  Serial.println("No GPS fix for 150 seconds, sending failure msg");
  txBuffer[0] = 0;
  txBuffer[1] = 0;
  txBuffer[2] = 0;
  txBuffer[3] = 0;
  txBuffer[4] = 0;
  txBuffer[5] = 0;
  txBuffer[6] = 0;
  txBuffer[7] = 0;
  txBuffer[8] = 0;
  do_send(&sendjob);

}

}

void lowPower() {
  gps.enableSleep();
  digitalWrite(VEXT_ON, LOW);
  // gps.setLowPower();
  // Set two wakeup sources: Timer for heartbeat, and interrupt for
  // motion detection from IMU
  #ifdef HAS_IMU
    rtc_gpio_pulldown_en(GPIO_NUM_4);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,1);
    delay(500);
  #endif
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(500);
  esp_deep_sleep_start();
}

void loop() {
    os_runloop_once();
}
