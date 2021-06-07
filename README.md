## Intro

This is a testbed for the [Feuchtraumabzweigdose PCB](https://github.com/stadtulm/feuchtraumabzweigdose), which can also be adapted for similar ESP32 based boards incorporating LoRaWan transmitter, switchable GPS, MPU9250 IMU etc. Can be configured to transmit either GPS location or Wifi SSIDs, depending on battery voltage and availability.

Derived from [radforschung/Lora-TTNMapper-T-Beam](https://github.com/radforschung/Lora-TTNMapper-T-Beam) which itself was derived from [sbiermann/Lora-TTNMapper-ESP32](https://github.com/sbiermann/Lora-TTNMapper-ESP32) and with some information/inspiration from [cyberman54/ESP32-Paxcounter](https://github.com/cyberman54/ESP32-Paxcounter) and [Edzelf/LoRa](https://github.com/Edzelf/LoRa). Modified and enhanced by dermatthias with sleep and OTAA features for this fork. Additional sleepytimes for the GPS module enabling lower-power-ish operations by stk, based on work by [JoepSchyns/Low_power_TTGO_T-Beam](https://github.com/JoepSchyns/Low_power_TTGO_T-beam) and helpful [ublox documentation in the UKHAS Wiki](https://ukhas.org.uk/guides:ublox_psm). The wifi scan functionality was derived from [stadtulm/Lora-Wifi-Location-Tracker](https://github.com/stadtulm/Lora-Wifi-Location-Tracker).

## Software dependencies

Arduino IDE [ESP32 extension](https://github.com/espressif/arduino-esp32)

[TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)

[MCCI LMIC Arduino](https://github.com/mcci-catena/arduino-lmic) This is a hell to configure. You need to edit `~/Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h` and define the “right” band, e.g. `CFG_eu868` (and comment out non-matching others)

[Bolderflight MPU9250](https://github.com/bolderflight/MPU9250) library if you want to wake the device through the MPU9250 IMU

## How to use

* Create a new TTN (v3) application and obtain an App EUI, Device EUI and App Key for your device. Use the contents of [ttn-decoder.js](ttn-decoder.js) as your payload decoder. (The firmware can, as well, be configured to be payload-compatible with [Lora-TTNMapper-T-Beam](https://github.com/radforschung/Lora-TTNMapper-T-Beam) or [Lortinchen-WifiTracker](https://github.com/stadtulm/Lora-Wifi-Location-Tracker) devices. Have a look at the PAYLOAD_TYPE option in [config.h](/feuchtraumabzweigdose/config.h)) 
* Update [ttnconfig.sample.h](/feuchtraumabzweigdose/ttnconfig.sample.h) with your keys and rename the file to `ttnconfig.h`
* In [config.h](/feuchtraumabzweigdose/config.h), specify the pinmap and location source(s) you want to use. Optionally, you can also enable the IMU wakeup, change the transmission interval or optimize some specific GPS settings.