#include <WiFi.h>
#include <ModbusTCP.h>

// WIFI CONFIG
#define WIFI_SSID "...."
#define WIFI_PASSWORD "...."

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

// ================= MODBUS TCP CLIENT =================
ModbusTCP mb;
// ubah sesuai IP Address Factory IO / PLC yang dijadikan sebagai server.
IPAddress modbusServer(192, 168, 158, 223);
const uint16_t modbusPort = 502;

unsigned long modbusPollTimer = 0;

#define MODBUS_POLL_INTERVAL 50

// IO CONFIG
#define OUT1_PIN 25
#define OUT2_PIN 26
#define OUT3_PIN 27

#define START_PIN 33
#define STOP_PIN 32

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define INPUT_UPDATE_MS 1000
#define OUTPUT_UPDATE_MS 20

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

// ================= WIFI EVENT HANDLER =================
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

// ================= START WIFI =================
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

// ================= NETWORK FSM =================
void updateNetwork() {

  switch (netState) {

    case NET_DISCONNECTED:

      if (
        millis() - reconnectTimer >= WIFI_RECONNECT_INTERVAL_MS) {

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

// ================= VARIABLES =================
bool START = false;
bool STOP = false;

bool MOTOR = false;

bool sensorNow = false;
bool sensorLast = false;

enum SystemState {
  IDLE,
  RUNNING,
  DONE
};

SystemState state = IDLE;

uint16_t counter = 0;
uint16_t last_counter = 0;
unsigned long serialTimer = 0;

// ================= SETUP =================
void setup() {

  Serial.begin(115200);

  // ================= WIFI =================
  WiFi.onEvent(onWiFiEvent);

  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  reconnectTimer =
    millis() - WIFI_RECONNECT_INTERVAL_MS;

  // ================= INPUT =================
  pinMode(
    START_PIN,
    INPUT_PULLUP);

  pinMode(
    STOP_PIN,
    INPUT_PULLUP);

  // ================= OUTPUT =================
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT3_PIN, OUTPUT);

  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT3_PIN, LOW);

  // ================= MODBUS CLIENT =================
  mb.client();

  Serial.println("System Ready");
}

// ================= LOOP =================
void loop() {

  // WIFI FSM
  updateNetwork();

  // MUST RUN ALWAYS
  mb.task();

  // CONNECT MODBUS SERVER
  if (netState == NET_CONNECTED) {

    if (!mb.isConnected(modbusServer)) {

      Serial.println("[MODBUS] Connecting...");

      mb.connect(modbusServer);
    } else {

      // NON BLOCKING POLLING
      if (millis() - modbusPollTimer >= MODBUS_POLL_INTERVAL) {

        modbusPollTimer = millis();

        processControl();
      }

      // ================= SERIAL MONITOR =================

      if (millis() - serialTimer >= 500) {

        serialTimer = millis();

        Serial.print("MOTOR: ");
        Serial.print(MOTOR);

        Serial.print(" | SENSOR: ");
        Serial.print(sensorNow);

        Serial.print(" | Counter: ");
        Serial.print(counter);

        Serial.print(" | CONNECTED: ");
        Serial.println(
          mb.isConnected(modbusServer));
      }
    }
  }
}


void processControl() {
  mb.readIsts(modbusServer, 0, &sensorNow, 1, nullptr, 1);
  // ================= BUTTON =================
  START =
    !digitalRead(START_PIN);

  STOP =
    !digitalRead(STOP_PIN);

  switch (state) {

    // ================= IDLE =================
    case IDLE:

      MOTOR = false;

      if (START) {

        sensorLast = true;  

        state = RUNNING;
      }
      break;

    // ================= RUNNING =================
    case RUNNING:

      MOTOR = true;

      // edge detection
      if (sensorNow && !sensorLast) {

        counter++;
      }

      sensorLast = sensorNow;

      if (counter >= 20) {

        state = DONE;
      }

      if (STOP) {

        state = IDLE;
      }
      break;

    // ================= DONE =================
    case DONE:

      MOTOR = false;

      // tunggu reset (START ulang)
      if (START) {

        counter = 0;
        sensorLast = true;

        state = RUNNING;
      }
      break;
  }

  // ================= OUTPUT =================
  digitalWrite(OUT1_PIN, !MOTOR);
  digitalWrite(OUT3_PIN, MOTOR);

  // ================= MODBUS =================
  mb.writeCoil(modbusServer, 0, MOTOR, nullptr, 1);
}
