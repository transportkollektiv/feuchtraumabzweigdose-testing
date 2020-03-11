## Intro

This is a testbed for the Feuchtraumabzweigdose board, incorporating switchable GPS, MPU9250 etc.


Derived from [radforschung/Lora-TTNMapper-T-Beam](https://github.com/radforschung/Lora-TTNMapper-T-Beam) which itself was derived from [sbiermann/Lora-TTNMapper-ESP32](https://github.com/sbiermann/Lora-TTNMapper-ESP32) and with some information/inspiration from [cyberman54/ESP32-Paxcounter](https://github.com/cyberman54/ESP32-Paxcounter) and [Edzelf/LoRa](https://github.com/Edzelf/LoRa).

Modified and enhanced by dermatthias with sleep and OTAA features for this fork. Additional sleepytimes for the GPS module enabling lower-power-ish operations by stk, based on work by [JoepSchyns/Low_power_TTGO_T-Beam](https://github.com/JoepSchyns/Low_power_TTGO_T-beam) and helpful [ublox documentation in the UKHAS Wiki](https://ukhas.org.uk/guides:ublox_psm). This also measures and transmits battery voltage.

## Software dependencies

Arduino IDE [ESP32 extension](https://github.com/espressif/arduino-esp32)

[TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)

[MCCI LMIC Arduino](https://github.com/mcci-catena/arduino-lmic) This is a hell to configure. You need to edit `~/Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h` and define the “right” band, e.g. `CFG_eu868` (and comment out non-matching others)

[Bolderflight MPU9250](https://github.com/bolderflight/MPU9250) library if you want to wake the device through the MPU9250 IMU

## Todolist

* save the GPS 'status' so that on next boot it gets a fix faster.
* Adapt the data send frequency based on current velocity : When not moving, an update per hour should be enough. ← prototype with cheap (1€) IMU board; in this case MPU 9250

