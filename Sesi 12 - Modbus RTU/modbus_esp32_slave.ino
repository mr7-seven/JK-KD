#include <ModbusRTU.h>

#define MODBUS_USB_SERIAL 1
#define MODBUS_HARDWARE_UART 2

#define MODBUS_MODE MODBUS_USB_SERIAL

#define SLAVE_ID 1
#define MODBUS_BAUDRATE 9600

#define RXD2 16
#define TXD2 17

#define OUT1_PIN 25
#define OUT2_PIN 26
#define OUT3_PIN 27

#define IR_SENSOR_PIN 34

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define INPUT_UPDATE_MS 1000
#define OUTPUT_UPDATE_MS 20

ModbusRTU mb;

HardwareSerial ModbusSerial(2);

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

void setup() {

#if MODBUS_MODE == MODBUS_USB_SERIAL

  Serial.begin(MODBUS_BAUDRATE);

  mb.begin(&Serial);

#elif MODBUS_MODE == MODBUS_HARDWARE_UART

  ModbusSerial.begin(
    MODBUS_BAUDRATE,
    SERIAL_8N1,
    RXD2,
    TXD2);

  mb.begin(&ModbusSerial);

#endif

  mb.slave(SLAVE_ID);

  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);

  mb.addIsts(DINPUT_IR_SENSOR, false);

  for (uint8_t i = 0; i < TOTAL_COILS; i++) {

    ledcAttach(
      outputPins[i],
      PWM_FREQ,
      PWM_RESOLUTION);

    ledcWrite(outputPins[i], 0);

    mb.addCoil(i, false);

    mb.addHreg(i, 0);
  }

  for (uint8_t i = 0; i < TOTAL_IREGS; i++) {

    mb.addIreg(i, 0);
  }

  randomSeed(esp_random());

  inputTimer = millis();
  outputTimer = millis();
}

void loop() {

  mb.task();

  updateDiscreteInputs();

  updateInputRegisters();

  updateOutputs();
}

void updateDiscreteInputs() {

  discreteInputs[DINPUT_IR_SENSOR] =
    digitalRead(IR_SENSOR_PIN);

  mb.Ists(
    DINPUT_IR_SENSOR,
    discreteInputs[DINPUT_IR_SENSOR]);
}

void updateInputRegisters() {

  if (millis() - inputTimer < INPUT_UPDATE_MS) return;

  inputTimer += INPUT_UPDATE_MS;

  inputRegs[IREG_VOLTAGE] = random(2200, 2401);

  inputRegs[IREG_CURRENT] = random(10, 151);

  inputRegs[IREG_POWER] = random(500, 3501);

  for (uint8_t i = 0; i < TOTAL_IREGS; i++) {

    mb.Ireg(i, inputRegs[i]);
  }
}

void updateOutputs() {

  if (millis() - outputTimer < OUTPUT_UPDATE_MS) return;

  outputTimer += OUTPUT_UPDATE_MS;
  // industrial-safe: PWM tidak akan keluar sebelum coil ON.
  for (uint8_t i = 0; i < TOTAL_COILS; i++) {

    coilState[i] = mb.Coil(i);

    holdingRegs[i] = constrain(
      mb.Hreg(i),
      0,
      255);

    uint8_t pwmValue =
      coilState[i] ? holdingRegs[i] : 0;

    ledcWrite(
      outputPins[i],
      pwmValue);
  }
}