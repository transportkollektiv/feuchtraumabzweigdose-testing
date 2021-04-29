#define FW_VERSION "20210426-0.5.1"
#define TRACKER 14

#include <esp_wifi.h>
#include <esp_sleep.h>
#include "config.h"
#include "gps.h"
#include "lora.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#ifdef DEBUG
  #define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */
  #define STATCOUNT 1
#else
  #ifdef HAS_IMU
    #define TIME_TO_SLEEP 1800
  #else
    #define TIME_TO_SLEEP 600
  #endif
  #define STATCOUNT 2
#endif

#if defined(HAS_IMU) && HAS_IMU == MPU9250
  // MPU Accelerometer for wake on motion
  #include "MPU9250.h"
  MPU9250 IMU(SPI,IMU_NCS);
  int status;
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR double prevLat = 0;
RTC_DATA_ATTR double prevLon = 0;
RTC_DATA_ATTR int statCount = 0;
double dist = 0;
bool hasFix = false;

gps gps;

void onEvent (ev_t ev) {
  lora_onEvent(ev);

  if (ev == EV_TXCOMPLETE) {
    lowPower();
  }
}

float getBatteryVoltage() {
  // first read of voltage; this is the least accurate and will be discarded.
  analogRead(BATTERY_VOLTAGE);

  // read voltage multiple times and average over all readings
  uint16_t readings = 0;
  for (int i = 0; i < 5; i++) {
    readings += analogRead(BATTERY_VOLTAGE);
  }
  float reading = readings / 5;

  // calculate: 
  // * analog read resolution: 10 bit (0-1023)
  // * operating voltage: 3.3V
  // * voltage divider: 1.2 MOhm resistor + 3.3 MOhm resistor
  return reading * (1 + 1.2 / 3.3) * (3.3 / 1023);
}

void setup() {

  //Turn off WiFi and Bluetooth
  //WiFi.mode(WIFI_OFF);
  //esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  btStop();
  
  // measure battery voltage
  adcAttachPin(BATTERY_VOLTAGE);
  analogReadResolution(10);
  float vBat = getBatteryVoltage();
  txBuffer[9] = (uint16_t)(vBat * 100) >> 8;
  txBuffer[10] = (uint16_t)(vBat * 100);
  
  // setup Vext pin, keep high until LoRa is setup and it is clear that voltage is sufficient
  pinMode(VEXT_ON, OUTPUT);
  digitalWrite(VEXT_ON, HIGH);
  
  Serial.begin(115200);
  delay(100);
  Serial.println(FW_VERSION);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  //Increment boot number and print it every reboot
  ++bootCount;
  #ifdef DEBUG
    Serial.println("Boot number: " + String(bootCount));
    Serial.println("RTC_seqnoUp: " + String(RTC_seqnoUp));
    Serial.println("Stationary Counter: " + String(statCount));    
    Serial.println("Battery Voltage: " + String(vBat)); 
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
  lora_init(bootCount);

  //TODO determine whether to send emergency message because of voltage



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

  // start gps
  digitalWrite(VEXT_ON, LOW);
  gps.init();
  gps.softwareReset();

  unsigned long startGpsTime = millis();
  while (millis() < startGpsTime + GPS_TIMEOUT)
  {
    if (gps.checkGpsFix())
    {
      dist = TinyGPSPlus::distanceBetween(gps.lat(), gps.lng(), prevLat, prevLon);

      if (dist > DISTANCE_MOVED || statCount > STATCOUNT)
      {
        Serial.println("Distance moved: " + String(dist));
        Serial.println("Time stationary: " + String(statCount * TIME_TO_SLEEP * uS_TO_S_FACTOR));
        if (dist <= DISTANCE_MOVED)
        {
          Serial.println("Sending because stationary for longer than max.");
        }
        statCount = 0;
        prevLat = gps.lat();
        prevLon = gps.lng();
        // Prepare upstream data transmission at the next possible time.
        gps.buildPacket(txBuffer);
        hasFix = true;
        lora_send();
      }
      else
      {
        Serial.println("Not sending, stationary.");
        Serial.println("Distance moved: " + String(dist));
        #ifndef HAS_IMU
        Serial.println("Time stationary: " + String(statCount * TIME_TO_SLEEP * uS_TO_S_FACTOR));
        #else
        Serial.println("Time stationary: " + String(statCount));
        #endif

        ++statCount;
        RTC_seqnoUp = LMIC.seqnoUp; //FIXME
        lowPower();
      }
      break;
    }
  }

  // turn off gps to save power
  gps.enableSleep();
  digitalWrite(VEXT_ON, HIGH);

  if (!hasFix)
  {
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
    lora_send();
  }
}

void lowPower() {

  // Ensure that external power supply is off
  digitalWrite(VEXT_ON, HIGH);

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
