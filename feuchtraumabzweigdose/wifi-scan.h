#ifndef WIFI_SCAN_H
#define WIFI_SCAN_H

#include <Arduino.h>
#include <WiFi.h>

struct minWifi
{
    int id;
    int rssi;
};

class wifi
{
public:
    int scan(byte *data);
    void disable();
private:
    static int wifiComparer(const void *w1, const void *w2);
};

#endif //WIFI_SCAN_H