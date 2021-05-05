#define FW_VERSION "20210503-1.0.0"
#define TRACKER 14

#include <esp_sleep.h>
#include "config.h"
#include "wifi-scan.h"
#include "gps.h"
#include "lora.h"

//Some config validation
#if !((defined(USE_GPS) && USE_GPS == AS_PRIMARY) || (defined(USE_WIFI) && USE_WIFI == AS_PRIMARY))
#error "Config Error: GPS or Wifi must be enabled and set as primary location source in config.h!"
#endif

#if PAYLOAD_TYPE == PAYLOAD_WIFITRACKER && defined(USE_GPS)
#error "Config Error: Deactivate GPS if you want payload compatibility with Wifi trackers!"
#endif

#if PAYLOAD_TYPE == PAYLOAD_TBEAM && defined(USE_WIFI)
#error "Config Error: Deactivate Wifi if you want payload compatibility with T-Beam GPS trackers!"
#endif

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

#ifdef DEBUG
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) */
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
MPU9250 IMU(SPI, IMU_NCS);
int status;
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int statCount = 0;
float vBat;
wifi wifi;

#ifdef USE_GPS
gps gps;
uint8_t gps_data[9];
RTC_DATA_ATTR double prevLat = 0;
RTC_DATA_ATTR double prevLon = 0;
RTC_DATA_ATTR double dist = 0;
#endif

#ifdef USE_WIFI
uint8_t wifi_data[53];
int wifi_data_len = 0;
#endif

typedef enum
{
  GET_LOCATION_SUCCESS,
  GET_LOCATION_STATIONARY,
  GET_LOCATION_FAILED,
  GET_LOCATION_NOT_USED
} get_location_result_t;

float getBatteryVoltage()
{
  // first read of voltage; this is the least accurate and will be discarded.
  analogRead(BATTERY_VOLTAGE);

  delay(100);

  // read voltage multiple times and average over all readings
  uint16_t readings = 0;
  for (int i = 0; i < 5; i++)
  {
    readings += analogRead(BATTERY_VOLTAGE);
  }
  float reading = readings / 5;

  // calculate:
  // * analog read resolution: 10 bit (0-1023)
  // * operating voltage: 3.3V
  // * voltage divider: 1.2 MOhm resistor + 3.3 MOhm resistor
  return reading * (1 + 1.2 / 3.3) * (3.3 / 1023);
}

#ifdef USE_WIFI
get_location_result_t getWIFILocation()
{
  wifi_data_len = wifi.scan(wifi_data);
  return ((wifi_data_len > 0) ? GET_LOCATION_SUCCESS : GET_LOCATION_FAILED);
}
#endif

#ifdef USE_GPS
get_location_result_t getGPSLocation()
{
  get_location_result_t returnVal = GET_LOCATION_FAILED;

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
        gps.buildPacket(gps_data);
        returnVal = GET_LOCATION_SUCCESS;
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
        returnVal = GET_LOCATION_STATIONARY;
      }
      break;
    }
  }

  // turn off gps to save power
  gps.enableSleep();
  digitalWrite(VEXT_ON, HIGH);

  if (returnVal == GET_LOCATION_FAILED)
  {
    Serial.println("No GPS fix for 150 seconds.");
  }

  return returnVal;
}
#endif

void onEvent(ev_t ev)
{
  lora_onEvent(ev);

  if (ev == EV_TXCOMPLETE)
  {
    lowPower();
  }
}

void lowPower()
{
  // Ensure that external power supply is off
  digitalWrite(VEXT_ON, HIGH);

// Set two wakeup sources: Timer for heartbeat, and interrupt for
// motion detection from IMU
#ifdef HAS_IMU
  rtc_gpio_pulldown_en(GPIO_NUM_4);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1);
  delay(500);
#endif
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(500);
  esp_deep_sleep_start();
}

void setup()
{

  //Turn off Bluetooth (and WiFi)
  btStop();
#ifndef USE_WIFI
  wifi.disable();
#endif

  // measure battery voltage
  adcAttachPin(BATTERY_VOLTAGE);
  analogReadResolution(10);
  vBat = getBatteryVoltage();

  // setup Vext pin, keep high until LoRa is setup and it is clear that voltage is sufficient
  pinMode(VEXT_ON, OUTPUT);
  digitalWrite(VEXT_ON, HIGH);

  // start Serial
  Serial.begin(115200);
  delay(100);
  Serial.println(FW_VERSION);

  ++bootCount; // increment boot number and print it every reboot
#ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  Serial.println("Boot number: " + String(bootCount));
  Serial.println("RTC_seqnoUp: " + String(RTC_seqnoUp));
  Serial.println("Stationary Counter: " + String(statCount));
  Serial.println("Battery Voltage: " + String(vBat));
  Serial.print("Real Battery Voltage from ADC: ");
  Serial.println(analogRead(BATTERY_VOLTAGE));

  // print the wakeup reason for ESP32
  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
#endif

//IMU init
#if defined(HAS_IMU) && HAS_IMU == MPU9250
  // start communication with IMU
  status = IMU.begin();
#ifdef DEBUG
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
#endif
  int imo_wom = IMU.enableWakeOnMotion(IMU_WAKEUP_FORCE, MPU9250::LP_ACCEL_ODR_15_63HZ);
  Serial.print("Wake on motion set: ");
  Serial.println(imo_wom);
#endif

  // LMIC init
  os_init();
  lora_init(bootCount);

  //TODO determine whether to send emergency message because of voltage

  // Get Locations
  get_location_result_t result_gps = GET_LOCATION_NOT_USED;
  get_location_result_t result_wifi = GET_LOCATION_NOT_USED;

  #if defined(USE_GPS) && USE_GPS == AS_PRIMARY
    result_gps = getGPSLocation();
    if (result_gps == GET_LOCATION_STATIONARY)
      lowPower();
  #endif

  #if defined(USE_WIFI) && USE_WIFI == AS_PRIMARY
    result_wifi = getWIFILocation();

  #elif defined(USE_WIFI) && USE_WIFI == AS_FALLBACK
    if (!(result_gps == GET_LOCATION_SUCCESS))
    {
      result_wifi = getWIFILocation();
    }
  #endif

  #if defined(USE_GPS) && USE_GPS == AS_FALLBACK
    if (!(result_wifi == GET_LOCATION_SUCCESS))
    {
      result_gps = getGPSLocation();
    }
  #endif

  #if PAYLOAD_TYPE == PAYLOAD_DEFAULT || PAYLOAD_TYPE == PAYLOAD_TBEAM
    uint8_t voltage_data[2];
    voltage_data[0] = (uint16_t)(vBat * 100) >> 8;
    voltage_data[1] = (uint16_t)(vBat * 100);

  #elif PAYLOAD_TYPE == PAYLOAD_WIFITRACKER
    uint8_t voltage_data;
    voltage_data = vBat * 1024 / (4 * 7.8);
  #endif

  

  if (result_gps == GET_LOCATION_SUCCESS && result_wifi == GET_LOCATION_SUCCESS)
  {
    #if PAYLOAD_TYPE == PAYLOAD_DEFAULT
    //Send both gps and wifi
    Serial.println("Sending location data from GPS and WIFI");
    uint8_t txBuffer[3 + 9 + wifi_data_len];
    txBuffer[0] = 0x10;
    memcpy(txBuffer + 1, voltage_data, 2);
    memcpy(txBuffer + 3, gps_data, 9);
    memcpy(txBuffer + 3 + 9, wifi_data, wifi_data_len);
    lora_send(txBuffer, sizeof(txBuffer));
    #endif
  }
  else if (result_gps == GET_LOCATION_SUCCESS)
  {
    //Send only gps
    Serial.println("Sending location data from GPS");

    #if PAYLOAD_TYPE == PAYLOAD_DEFAULT
    uint8_t txBuffer[3 + 9];
    txBuffer[0] = 0x11;
    memcpy(txBuffer + 1, voltage_data, 2);
    memcpy(txBuffer + 3, gps_data, 9);
    lora_send(txBuffer, sizeof(txBuffer));

    #elif PAYLOAD_TYPE == PAYLOAD_TBEAM
    uint8_t txBuffer[9 + 2];
    memcpy(txBuffer, gps_data, 9);
    memcpy(txBuffer + 9, voltage_data, 2);
    lora_send(txBuffer, sizeof(txBuffer));
    #endif

    
  }
  else if (result_wifi == GET_LOCATION_SUCCESS)
  {
    //Send only wifi
    Serial.println("Sending location data from WIFI");

    #if PAYLOAD_TYPE == PAYLOAD_DEFAULT
    uint8_t txBuffer[3 + wifi_data_len];
    txBuffer[0] = 0x12;
    memcpy(txBuffer + 1, voltage_data, 2);
    memcpy(txBuffer + 3, wifi_data, wifi_data_len);
    lora_send(txBuffer, sizeof(txBuffer));

    #elif PAYLOAD_TYPE == PAYLOAD_WIFITRACKER
    uint8_t txBuffer[2 + wifi_data_len];
    txBuffer[0] = 0x02;
    txBuffer[1] = voltage_data;
    memcpy(txBuffer + 2, wifi_data, wifi_data_len);
    lora_send(txBuffer, sizeof(txBuffer));
    #endif

    
  }
  else
  {
    //Send failure message
    Serial.println("Sending failure message");

    #if PAYLOAD_TYPE == PAYLOAD_DEFAULT
    uint8_t txBuffer[3];
    txBuffer[0] = 0x01;
    memcpy(txBuffer + 1, voltage_data, 2);

    #elif PAYLOAD_TYPE == PAYLOAD_TBEAM
    uint8_t txBuffer[9 + 2];
    for (int i = 0; i < 9; i++) gps_data[i] = 0x00;
    memcpy(txBuffer, gps_data, 9);
    memcpy(txBuffer + 9, voltage_data, 2);

    #elif PAYLOAD_TYPE == PAYLOAD_WIFITRACKER
    uint8_t txBuffer[2];
    txBuffer[0] = 0x02;
    txBuffer[1] = voltage_data;
    #endif

    lora_send(txBuffer, sizeof(txBuffer));
  }
  
}

void loop()
{
  os_runloop_once();
}
