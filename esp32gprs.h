#ifndef SRC_SUPLA_NETWORK_ESP32GPRS_H_
#define SRC_SUPLA_NETWORK_ESP32GPRS_H_

/* In the main code, include:
#include "esp32gprs.h",
and declare:
Supla::GPRS gprs;
*/

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1
#define BAUDRATE         115200
#define MODEM_RST             5
#define MODEM_PWRKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

const char apn[] = ""; 
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <Wire.h>

#include <supla/network/network.h>
#include <supla/network/client.h>
#include <supla/supla_lib_config.h>
#include <supla/log_wrapper.h>

void setupModem()
{

   #ifdef MODEM_RST
    Serial.println("Setting MODEM_RST pin to HIGH (Reset disabled).");
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    Serial.println("Configuring MODEM_PWRKEY and MODEM_POWER_ON as OUTPUT.");
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    Serial.println("Turning on the Modem power.");
    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    Serial.println("PWRKEY cycle start: HIGH -> LOW -> HIGH to power on the modem.");
    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    Serial.println("PWRKEY pulled LOW for 1 second.");
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);
    Serial.println("PWRKEY set to HIGH again. Modem should be on now.");  
}

void initializeModem() {
  SerialAT.begin(BAUDRATE, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  SerialMon.println(F("Restarting modem..."));
  modem.restart(); 
  
  String modemInfo = modem.getModemInfo();
  if (modemInfo.length() > 0) {
    SerialMon.println("Modem Info: " + modemInfo);
  } else {
    SerialMon.println(F("Failed to initialize modem"));
  }
}

void connectToNetwork() {
  SerialMon.println(F("Connecting to network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("Fail to connect to network"));
    return;
  }
  SerialMon.println(F("Network connected"));

  
  SerialMon.println(F("Connecting to GPRS..."));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(F("Fail to connect to GPRS"));
    return;
  }
  SerialMon.println(F("GPRS connected"));
}

static bool gprs_connected = false;

namespace Supla {
class GPRSClient : public Supla::Client {
public:
  TinyGsmClient modemClient;

  explicit GPRSClient(TinyGsm &modem)
    : modemClient(modem) {}

  ~GPRSClient() override {}

  int connectImp(const char *host, uint16_t port) override {
    return modemClient.connect(host, port) ? 1 : 0;
  }

  size_t writeImp(const uint8_t *buf, size_t size) override {
    return modemClient.write(buf, size);
  }

  int readImp(uint8_t *buf, size_t size) override {
    return modemClient.available() ? modemClient.read(buf, size) : -1;
  }

  int available() override {
    return modemClient.available();
  }

  void stop() override {
    modemClient.stop();
  }

  uint8_t connected() override {
    return modemClient.connected();
  }

  void setTimeoutMs(uint16_t timeoutMs) override {
    // Implement if applicable
  }
};

class GPRS : public Supla::Network {
public:
  TinyGsm modem;

  GPRS()
    : Network(nullptr), modem(SerialAT) {
    intfType = IntfType::Ethernet;  // Use Ethernet for GPRS for compatibility
  }

  bool isReady() override {
    return modem.isGprsConnected();
  }

  void setup() override {

    Serial.println(F("[GPRS] Setting up GPRS connection..."));
    setSSLEnabled(false);
    setupModem();        
    initializeModem();   
    connectToNetwork();  

    
    if (!modem.waitForNetwork()) {
      Serial.println(F("[GPRS] Failed to connect to network."));
      return;
    }
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      Serial.println(F("[GPRS] Failed to connect to GPRS."));
      return;
    }
    Serial.println(F("[GPRS] GPRS connected successfully."));
    gprs_connected = true;
  }

  void disable() override {
    modem.gprsDisconnect();
    Serial.println(F("[GPRS] GPRS disconnected."));
    gprs_connected = false;
  }

  void fillStateData(TDSC_ChannelState *channelState) override {
    // Implement as needed
  }

  Supla::Client *createClient() override {
    return new GPRSClient(modem);
  }
};

};
#endif
