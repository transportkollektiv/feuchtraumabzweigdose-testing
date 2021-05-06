#include "wifi-scan.h"

int wifi::wifiComparer(const void * w1, const void * w2) {
  const struct minWifi *elem1 = (minWifi*)w1;    
  const struct minWifi *elem2 = (minWifi*)w2;
  return ( elem2->rssi - elem1->rssi );
}

void wifi::disable() {
    WiFi.mode(WIFI_OFF);
  //esp_wifi_stop();
  //esp_wifi_set_mode(WIFI_MODE_NULL);
}

int wifi::scan(byte* data) {
  Serial.println("WIFI: Start Scan");
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, false);
  
  //we dont need the wifi anymore
  WiFi.setSleep(true);

  String ssid;
  uint8_t encryptionType;
  int32_t RSSI;
  uint8_t* BSSID;
  int32_t channel;

  minWifi wifilist[n];
  for (int i = 0; i < n; i++) {
    WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel);
    wifilist[i] = {i, RSSI};
  }
  qsort(wifilist, n, sizeof(minWifi), wifiComparer);

  int datalen = 0;
  for (int i = 0; i < n; i++)
  {
    WiFi.getNetworkInfo(wifilist[i].id, ssid, encryptionType, RSSI, BSSID, channel);
    Serial.print("WIFI: Found ");
    Serial.println(ssid);
    memcpy(data+i*6, BSSID, 6);
    datalen = i*6+6;
    if (i==7) {
      break;
    }
  }

  Serial.println("WIFI: Scan finished");

  return datalen;
}

