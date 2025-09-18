/* Real-Time Solar Energy Monitoring System
   - Platform: ESP32
   - Sensors: INA219 (I2C) OR analog voltage divider, ACS712 (analog current), DS18B20 (OneWire)
   - Connectivity: 4G modem (TinyGSM + HTTP)
   - RTOS: FreeRTOS tasks for sensor reading, control, and comms
   - Author: Pranav (example)
*/

/* -------------------- Libraries -------------------- */
#include <Wire.h>
#include <Adafruit_INA219.h>         // optional — comment out if not using INA219
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGsmClient.h>           // for 4G modem (TinyGSM)
#include <HTTPClient.h>
#include <Arduino.h>

/* -------------------- Configuration -------------------- */

// Choose sensor mode: set true to use INA219 for voltage/current, false to use analog readings + ACS712
#define USE_INA219 true

// Pins (change according to your wiring)
const int PIN_ADC_VOLTAGE = 35; // ADC pin for voltage divider (only if not using INA219)
const int PIN_ADC_CURRENT = 34; // ACS712 analog output
const int PIN_DS18B20 = 4;      // OneWire pin for DS18B20
const int PIN_PWM_CHARGE = 26;  // PWM output to MOSFET gate (charge controller)
const int PWM_FREQ = 5000;      // PWM frequency
const int PWM_CHANNEL = 0;
const int PWM_RES = 8;          // resolution (8-bit)

// INA219 instance
Adafruit_INA219 ina219;

// OneWire + DS18B20
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ADC calibration (if using analog measurement)
// These constants must be calibrated for your divider and ADC reference
const float ADC_MAX = 4095.0;     // 12-bit ADC for ESP32
const float VREF = 3.3;           // reference voltage (V)
const float VOLTAGE_DIV_RATIO = 11.0; // e.g., if divider scales 0-55V down to 0-5V, ratio = 11
const float ACS712_SENSITIVITY = 0.066; // V/A (example: 66mV/A for 30A module) - check datasheet
const float ACS712_ZERO_VOLT = VREF / 2.0; // baseline at no current (depends on wiring)

// 4G modem (example using TinyGSM and a generic modem)
#define TINY_GSM_MODEM_SIM7600   // choose your modem; change if different
#include <TinyGsmClient.h>
#ifdef TINY_GSM_MODEM_SIM7600
  // If using SIM7600, chipset-specific defines might be needed
#endif

// Replace with your modem's serial pins and APN
HardwareSerial SerialAT(1); // UART1 for modem
const int MODEM_RX = 16; // ESP32 RX -> Modem TX
const int MODEM_TX = 17; // ESP32 TX -> Modem RX
const char apn[] = "your_apn_here";
const char serverUrl[] = "http://your-server.example.com/api/solar"; // endpoint to POST telemetry

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

/* -------------------- Shared data (thread-safe via simple protection) -------------------- */
volatile float latestVoltage = 0.0;
volatile float latestCurrent = 0.0;
volatile float latestPower = 0.0;
volatile float latestTemp = 0.0;

// Simple mutex-ish with portENTER_CRITICAL / portEXIT_CRITICAL
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

/* -------------------- Helper functions -------------------- */

// Read voltage: either from INA219 or ADC divider
float measureVoltage() {
#if USE_INA219
  float busV = ina219.getBusVoltage_V(); // direct reading from INA219
  return busV;
#else
  uint32_t raw = analogRead(PIN_ADC_VOLTAGE);
  float vMeasured = (raw / ADC_MAX) * VREF;
  float actualVoltage = vMeasured * VOLTAGE_DIV_RATIO;
  return actualVoltage;
#endif
}

// Read current: INA219 or ACS712 analog
float measureCurrent() {
#if USE_INA219
  float current_mA = ina219.getCurrent_mA(); // mA
  return current_mA / 1000.0; // convert to A
#else
  uint32_t raw = analogRead(PIN_ADC_CURRENT);
  float vMeasured = (raw / ADC_MAX) * VREF;
  float voltageAcross = vMeasured - ACS712_ZERO_VOLT;
  float current = voltageAcross / ACS712_SENSITIVITY; // A (signed)
  return current;
#endif
}

// Read temperature
float measureTemperature() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return t;
}

/* -------------------- FreeRTOS Tasks -------------------- */

// Sensor task: read sensors periodically and update shared variables
void sensorTask(void *pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(2000); // 2s
  while (true) {
    float V = measureVoltage();
    float I = measureCurrent();
    float T = measureTemperature();
    float P = V * I;

    // Update shared values safely
    portENTER_CRITICAL(&dataMux);
    latestVoltage = V;
    latestCurrent = I;
    latestPower = P;
    latestTemp = T;
    portEXIT_CRITICAL(&dataMux);

    vTaskDelay(delayTicks);
  }
}

// Control task: simple PWM-based charge control using thresholding
void controlTask(void *pvParameters) {
  // Example logic: if battery voltage < threshold -> enable charging (PWM duty)
  const float BATTERY_CHARGE_THRESHOLD = 12.4; // example for 12V system
  const TickType_t delayTicks = pdMS_TO_TICKS(1000);
  while (true) {
    float Vlocal;
    portENTER_CRITICAL(&dataMux);
    Vlocal = latestVoltage;
    portEXIT_CRITICAL(&dataMux);

    uint32_t duty = 0;
    if (Vlocal < BATTERY_CHARGE_THRESHOLD) {
      // simple proportional control: more voltage -> less duty
      float diff = BATTERY_CHARGE_THRESHOLD - Vlocal;
      float dutyF = diff * 20.0; // tune factor -> get duty 0..255
      if (dutyF > 255) dutyF = 255;
      duty = (uint32_t)dutyF;
    } else {
      duty = 0;
    }
    ledcWrite(PWM_CHANNEL, duty); // set PWM duty (0..255 for 8-bit)

    vTaskDelay(delayTicks);
  }
}

// Comms task: connect modem and post telemetry periodically
void commsTask(void *pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(10000); // send every 10s (tune as needed)

  // Initialize modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);
  if (!modem.restart()) {
    // modem restart failed - handle as needed
  }
  // Connect to operator
  bool connected = modem.gprsConnect(apn, "", "");
  if (!connected) {
    // handle failure; you might retry in loop
  }

  while (true) {
    float V, I, P, T;
    portENTER_CRITICAL(&dataMux);
    V = latestVoltage; I = latestCurrent; P = latestPower; T = latestTemp;
    portEXIT_CRITICAL(&dataMux);

    // Build JSON or form data
    String payload = "{\"voltage\":" + String(V, 3) +
                     ",\"current\":" + String(I, 3) +
                     ",\"power\":" + String(P, 3) +
                     ",\"temp\":" + String(T, 2) + "}";

    // HTTP POST using TinyGSMClient + HTTPClient
    if (modem.isGprsConnected()) {
      HTTPClient http;
      // Use TinyGsmClient as transport for HTTPClient
      // Note: On some platforms you need to use a dedicated HTTP client that accepts a client
      // Example using ArduinoHttpClient library is more appropriate, but using HTTPClient with WiFiClient is typical.
      // We'll demonstrate a basic TinyGSM client POST flow:
      if (client.connect(serverUrl, 80)) {
        // For raw HTTP using TinyGSMClient:
        String req = String("POST ") + serverUrl + " HTTP/1.1\r\n";
        req += "Host: your-server.example.com\r\n";
        req += "Content-Type: application/json\r\n";
        req += "Content-Length: " + String(payload.length()) + "\r\n\r\n";
        req += payload;
        client.print(req);
        // Optionally, read response
        // while(client.available()) { String line = client.readStringUntil('\n'); }
        client.stop();
      } else {
        // Connection fail
      }
    } else {
      // Not connected - attempt reconnect
      modem.gprsConnect(apn, "", "");
    }

    vTaskDelay(delayTicks);
  }
}

/* -------------------- Setup & Loop -------------------- */

void setup() {
  Serial.begin(115200);
  Wire.begin(); // I2C

  // Initialize sensors
#if USE_INA219
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    // fallback or continue
  }
#endif
  sensors.begin();

  // PWM setup
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_CHARGE, PWM_CHANNEL);

  // Create tasks pinned to core (optional)
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(commsTask, "CommsTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // Loop does nothing, tasks run in FreeRTOS
  delay(1000);
}

