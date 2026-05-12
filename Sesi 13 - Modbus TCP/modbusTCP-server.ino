#include <WiFi.h>
#include <ModbusTCP.h>

// NETWORK CONFIG
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

//
// 1 = DHCP
// 0 = STATIC IP
//
#define USE_DHCP 1

#if !USE_DHCP

IPAddress localIP(192, 168, 1, 50);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(1, 1, 1, 1);

#endif

// MODBUS TCP
#define MODBUS_PORT 502

ModbusTCP mb;

// IO CONFIG
#define OUT1_PIN 25
#define OUT2_PIN 26
#define OUT3_PIN 27

#define IR_SENSOR_PIN 34

#define PWM_FREQ        5000
#define PWM_RESOLUTION  8

#define INPUT_UPDATE_MS   1000
#define OUTPUT_UPDATE_MS  20

// WIFI FSM
enum NetworkState {
  NET_DISCONNECTED,
  NET_CONNECTING,
  NET_CONNECTED
};

volatile NetworkState netState =
  NET_DISCONNECTED;

bool wifiBeginStarted = false;

unsigned long reconnectTimer = 0;

#define WIFI_RECONNECT_INTERVAL_MS 5000

// MODBUS MAP
enum CoilAddress {
  COIL_OUT1,
  COIL_OUT2,
  COIL_OUT3,
  TOTAL_COILS
};

enum DiscreteInputAddress {
  DINPUT_IR_SENSOR,
  TOTAL_DINPUTS
};

enum InputRegisterAddress {
  IREG_VOLTAGE,
  IREG_CURRENT,
  IREG_POWER,
  TOTAL_IREGS
};

enum HoldingRegisterAddress {
  HREG_PWM1,
  HREG_PWM2,
  HREG_PWM3,
  TOTAL_HREGS
};

// VARIABLES
const uint8_t outputPins[TOTAL_COILS] = {
  OUT1_PIN,
  OUT2_PIN,
  OUT3_PIN
};

bool coilState[TOTAL_COILS];

bool discreteInputs[TOTAL_DINPUTS];

uint16_t inputRegs[TOTAL_IREGS];

uint16_t holdingRegs[TOTAL_HREGS];

unsigned long inputTimer = 0;
unsigned long outputTimer = 0;

// WIFI EVENT HANDLER
void onWiFiEvent(
  WiFiEvent_t event,
  WiFiEventInfo_t info) {

  switch (event) {

    case ARDUINO_EVENT_WIFI_STA_START:

      Serial.println("[WiFi] STA Started");

      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:

      Serial.println("[WiFi] Connected to AP");

      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:

      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());

      netState = NET_CONNECTED;

      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:

      Serial.print("[WiFi] Disconnected. Reason: ");
      Serial.println(
        info.wifi_sta_disconnected.reason);

      netState = NET_DISCONNECTED;

      wifiBeginStarted = false;

      reconnectTimer = millis();

      break;

    default:

      break;
  }
}

// START WIFI
void startWiFi() {

  if (wifiBeginStarted) return;

  wifiBeginStarted = true;

  netState = NET_CONNECTING;

  Serial.println("[WiFi] Connecting...");

#if !USE_DHCP

  bool configOK =
    WiFi.config(
      localIP,
      gateway,
      subnet,
      dns1,
      dns2);

  if (!configOK) {

    Serial.println(
      "[WiFi] Static IP Config Failed");
  }

#endif

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD);
}

// NETWORK FSM
void updateNetwork() {

  switch (netState) {

    case NET_DISCONNECTED:

      if (
        millis() - reconnectTimer >=
        WIFI_RECONNECT_INTERVAL_MS) {

        startWiFi();
      }

      break;

    case NET_CONNECTING:

      //
      // handled by event
      //
      break;

    case NET_CONNECTED:

      //
      // extra safety
      //
      if (
        WiFi.status() != WL_CONNECTED) {

        Serial.println(
          "[WiFi] Status Mismatch");

        netState = NET_DISCONNECTED;

        wifiBeginStarted = false;

        reconnectTimer = millis();
      }

      break;
  }
}

// SETUP
void setup() {

  Serial.begin(115200);

  // WIFI
  WiFi.onEvent(onWiFiEvent);

  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  reconnectTimer =
    millis() - WIFI_RECONNECT_INTERVAL_MS;

  mb.server(MODBUS_PORT);

  // INPUT
  pinMode(
    IR_SENSOR_PIN,
    INPUT_PULLUP);

  // DISCRETE INPUT
  mb.addIsts(
    DINPUT_IR_SENSOR,
    false);

  // OUTPUT + HOLDING REGISTER
  for (uint8_t i = 0;
       i < TOTAL_COILS;
       i++) {

    ledcAttachChannel(
      outputPins[i],
      PWM_FREQ,
      PWM_RESOLUTION,
      i);

    ledcWrite(
      outputPins[i],
      0);

    mb.addCoil(i, false);

    mb.addHreg(i, 0);
  }

  // INPUT REGISTER
  for (uint8_t i = 0;
       i < TOTAL_IREGS;
       i++) {

    mb.addIreg(i, 0);
  }

  randomSeed(esp_random());

  inputTimer = millis();
  outputTimer = millis();

  Serial.println("System Ready");
}

// LOOP
void loop() {

  // WIFI FSM
  updateNetwork();

  mb.task();

  updateDiscreteInputs();

  updateInputRegisters();

  updateOutputs();
}

// DISCRETE INPUTS
void updateDiscreteInputs() {

  discreteInputs[DINPUT_IR_SENSOR] =
    digitalRead(IR_SENSOR_PIN);

  mb.Ists(
    DINPUT_IR_SENSOR,
    discreteInputs[DINPUT_IR_SENSOR]);
}

// INPUT REGISTERS
void updateInputRegisters() {

  if (
    millis() - inputTimer <
    INPUT_UPDATE_MS) return;

  inputTimer += INPUT_UPDATE_MS;

  // dummy data
  inputRegs[IREG_VOLTAGE] =
    random(2200, 2401);

  inputRegs[IREG_CURRENT] =
    random(10, 151);

  inputRegs[IREG_POWER] =
    random(500, 3501);

  for (uint8_t i = 0;
       i < TOTAL_IREGS;
       i++) {

    mb.Ireg(
      i,
      inputRegs[i]);
  }
}

// OUTPUTS
void updateOutputs() {

  if (
    millis() - outputTimer <
    OUTPUT_UPDATE_MS) return;

  outputTimer += OUTPUT_UPDATE_MS;

  // PWM active only if Coil ON
  for (uint8_t i = 0;
       i < TOTAL_COILS;
       i++) {

    coilState[i] =
      mb.Coil(i);

    holdingRegs[i] =
      constrain(
        mb.Hreg(i),
        0,
        255);

    uint8_t pwmValue =
      coilState[i]
      ? holdingRegs[i]
      : 0;

    ledcWrite(
      outputPins[i],
      pwmValue);
  }
}