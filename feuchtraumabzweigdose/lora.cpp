#include "lora.h"

RTC_DATA_ATTR u4_t RTC_seqnoUp = 0;
RTC_DATA_ATTR int otaaJoined = 0;
RTC_DATA_ATTR u1_t keep_nwkKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
RTC_DATA_ATTR u1_t keep_artKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
RTC_DATA_ATTR devaddr_t keep_devaddr = 0;

osjob_t sendjob;
char s[32]; // used to sprintf for Serial output

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RFM_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM_RESET,
    .dio = {RFM_DIO0, RFM_DIO1, LMIC_UNUSED_PIN},
};

// OTAA, see config.h for settings
#if defined(USE_OTAA) && USE_OTAA == true
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#else
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#endif

void lora_init(int bootCount)
{
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // ABP or OTAA
  #if defined(USE_OTAA) && USE_OTAA == true
    #ifdef DEBUG
    Serial.println("Activation method: OTAA");
    #endif
    if (bootCount > 0 && otaaJoined == 1)
    {
      Serial.println(F("Restoring OTAA session credentials from memory"));
      memcpy(LMIC.nwkKey, keep_nwkKey, 16);
      memcpy(LMIC.artKey, keep_artKey, 16);
      LMIC.devaddr = keep_devaddr;
    }
  #else
    #ifdef DEBUG
    Serial.println("Activation method: ABP");
    #endif
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14);

  LMIC.seqnoUp = RTC_seqnoUp;
  LMIC.seqnoDn = RTC_seqnoUp;
}

void lora_onEvent(ev_t ev)
{
  switch (ev)
  {
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

    for (int i = 0; i < 16; i++)
    {
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
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      Serial.println(F("Received Ack"));
    }
    if (LMIC.dataLen)
    {
      sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
      Serial.println(s);
      sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
      Serial.println(s);
    }
    RTC_seqnoUp = LMIC.seqnoUp;
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
    Serial.println((unsigned)ev);
    break;
  }
}

void lora_send (uint8_t *txBuffer, size_t txBuffer_size) {
  do_send(&sendjob, txBuffer, txBuffer_size);
}

void do_send(osjob_t *j, uint8_t *txBuffer, size_t txBuffer_size)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    LMIC_setTxData2(1, txBuffer, txBuffer_size, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
