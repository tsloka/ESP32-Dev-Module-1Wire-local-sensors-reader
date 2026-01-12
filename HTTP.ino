#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ETH.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* ===================== CONFIG ===================== */

#define BUS_COUNT              3
#define MAX_SENSORS_PER_BUS   10
#define MAX_SENSORS           (BUS_COUNT * MAX_SENSORS_PER_BUS)

static const uint8_t BUS_PINS[BUS_COUNT] = {14, 16, 32};

#define POLL_INTERVAL_MS      2000
#define HTTP_PORT             80

/* ======= ESP32-STICK-POE-P (FROZEN) ======= */

void startEthernet() {
  ETH.begin(
    ETH_PHY_LAN8720,
    1,      // PHY address
    23,     // MDC
    18,     // MDIO
    -1,     // no reset
    ETH_CLOCK_GPIO17_OUT
  );
}

/* ===================== DATA ===================== */

struct SensorSnapshot {
  uint8_t  bus;
  uint8_t  rom[8];
  char     romHex[17];
  uint64_t grentonId;
  float    temperatureC;
  bool     crcOk;
  uint32_t faults;
};

SensorSnapshot sensors[MAX_SENSORS];
volatile uint8_t sensorCount = 0;

SemaphoreHandle_t snapshotMutex;

/* ===================== BUSES ===================== */

OneWire* oneWire[BUS_COUNT];
DallasTemperature* dallas[BUS_COUNT];

/* ===================== HTTP ===================== */

WebServer server(HTTP_PORT);
volatile bool ethReady = false;

/* ===================== UTIL ===================== */

void romToHex(const uint8_t rom[8], char out[17]) {
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    out[i * 2]     = hex[(rom[i] >> 4) & 0x0F];
    out[i * 2 + 1] = hex[rom[i] & 0x0F];
  }
  out[16] = 0;
}

uint64_t romToGrenton(const uint8_t rom[8]) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) {
    v = (v << 8) | rom[i];
  }
  return v;
}

/* ===================== ENUMERATION ===================== */

void enumerateBus(uint8_t bus) {
  oneWire[bus]->reset_search();

  uint8_t rom[8];
  uint8_t countBefore = sensorCount;

  while (oneWire[bus]->search(rom)) {
    if (sensorCount >= MAX_SENSORS) break;

    SensorSnapshot &s = sensors[sensorCount];

    memcpy(s.rom, rom, 8);
    s.bus = bus;
    s.crcOk = (OneWire::crc8(rom, 7) == rom[7]);
    s.temperatureC = NAN;
    s.faults = 0;

    romToHex(rom, s.romHex);
    s.grentonId = romToGrenton(rom);

    Serial.printf(
      "Bus %u ENUM  ROM=%s  GrentonID=%llu  CRC=%s\n",
      bus, s.romHex, s.grentonId, s.crcOk ? "OK" : "BAD"
    );

    sensorCount++;

    if ((sensorCount - countBefore) >= MAX_SENSORS_PER_BUS) break;
  }

  Serial.printf(
    "Bus %u: %u sensors\n",
    bus, sensorCount - countBefore
  );
}

void enumerateAll() {
  sensorCount = 0;
  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    enumerateBus(b);
  }
}

/* ===================== SENSOR TASK ===================== */

void sensorTask(void*) {
  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    dallas[b]->setWaitForConversion(false);
    dallas[b]->setResolution(12);
  }

  for (;;) {
    for (uint8_t b = 0; b < BUS_COUNT; b++) {
      dallas[b]->requestTemperatures();
    }

    vTaskDelay(pdMS_TO_TICKS(750)); // 12-bit conversion

    xSemaphoreTake(snapshotMutex, portMAX_DELAY);

    for (uint8_t i = 0; i < sensorCount; i++) {
      SensorSnapshot &s = sensors[i];

      float t = dallas[s.bus]->getTempC(s.rom);
      if (t == DEVICE_DISCONNECTED_C) {
        s.faults++;
        s.temperatureC = NAN;
      } else {
        s.temperatureC = t;
      }

      Serial.printf(
        "Bus %u READ  ROM=%s  GrentonID=%llu  Temp=%.3f C\n",
        s.bus, s.romHex, s.grentonId, s.temperatureC
      );
    }

    xSemaphoreGive(snapshotMutex);
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

/* ===================== HTTP ===================== */

void handleSensors() {
  if (!ethReady) {
    server.send(503, "application/json", "{\"error\":\"ETH down\"}");
    return;
  }

  StaticJsonDocument<2048> doc;
  JsonArray arr = doc.createNestedArray("sensors");

  xSemaphoreTake(snapshotMutex, portMAX_DELAY);

  for (uint8_t i = 0; i < sensorCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["bus"] = sensors[i].bus;
    o["rom_hex"] = sensors[i].romHex;
    o["grenton_id"] = sensors[i].grentonId;
    o["temperature_c"] = sensors[i].temperatureC;
    o["crc_ok"] = sensors[i].crcOk;
    o["faults"] = sensors[i].faults;
  }

  xSemaphoreGive(snapshotMutex);

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

/* ===================== SETUP ===================== */

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Booting ESP32-STICK-POE-P");

  snapshotMutex = xSemaphoreCreateMutex();

  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    oneWire[b] = new OneWire(BUS_PINS[b]);
    dallas[b]  = new DallasTemperature(oneWire[b]);
  }

  enumerateAll();

  xTaskCreatePinnedToCore(
    sensorTask,
    "sensorTask",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );

  startEthernet();

  while (!ETH.linkUp()) {
    delay(100);
  }

  ethReady = true;
  Serial.print("ETH IP: ");
  Serial.println(ETH.localIP());

  server.on("/sensors", HTTP_GET, handleSensors);
  server.begin();

  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
