#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BUS_COUNT 3

// ===== CHANGE PINS IF NEEDED =====
static const uint8_t BUS_PINS[BUS_COUNT] = {
  14,  // Bus 0
  16,  // Bus 1
  32   // Bus 2
};

#define MAX_SENSORS_TOTAL 64
#define POLL_INTERVAL_MS 3000   // safe for up to 30 sensors @ 12-bit

// ================================

struct SensorInfo {
  uint8_t bus;
  DeviceAddress rom;
  uint64_t grentonId;
  bool crcOk;
  float temperatureC;
};

SensorInfo sensors[MAX_SENSORS_TOTAL];
uint8_t sensorCount = 0;

OneWire* oneWire[BUS_COUNT];
DallasTemperature* dallas[BUS_COUNT];

// ---------- Utilities ----------

String romToHex(const DeviceAddress rom) {
  char buf[17];
  for (uint8_t i = 0; i < 8; i++) {
    sprintf(&buf[i * 2], "%02X", rom[i]);
  }
  return String(buf);
}

uint64_t romToGrentonId(const DeviceAddress rom) {
  uint64_t id = 0;
  for (uint8_t i = 1; i <= 6; i++) {
    id = (id << 8) | rom[i];
  }
  return id;
}

void hardResetBus(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(600);
  pinMode(pin, INPUT_PULLUP);
  delay(10);
}

// ---------- Enumeration ----------

void enumerateBus(uint8_t busIndex) {
  Serial.printf("\n--- Enumerating bus %u (GPIO %u) ---\n",
                busIndex, BUS_PINS[busIndex]);

  hardResetBus(BUS_PINS[busIndex]);

  DeviceAddress rom;
  oneWire[busIndex]->reset_search();

  uint8_t found = 0;

  while (oneWire[busIndex]->search(rom)) {
    if (sensorCount >= MAX_SENSORS_TOTAL) break;

    SensorInfo& s = sensors[sensorCount++];
    s.bus = busIndex;
    memcpy(s.rom, rom, 8);
    s.crcOk = (OneWire::crc8(rom, 7) == rom[7]);
    s.grentonId = romToGrentonId(rom);
    s.temperatureC = NAN;

    Serial.printf(
      "Bus %u Sensor %u | ROM=%s | GID=%llu | CRC=%s\n",
      busIndex,
      found,
      romToHex(rom).c_str(),
      s.grentonId,
      s.crcOk ? "OK" : "BAD"
    );

    found++;
    delay(3);
  }

  Serial.printf("Bus %u enumeration complete (%u sensors)\n",
                busIndex, found);
}

void enumerateAllBuses() {
  sensorCount = 0;

  Serial.println("\n=========== ENUMERATION START ===========");

  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    enumerateBus(b);
    delay(50);
  }

  Serial.println("\n=========== ENUMERATION DONE ===========");
  Serial.printf("Total sensors found: %u\n\n", sensorCount);
}

// ---------- Polling ----------

void pollSensors() {
  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    dallas[b]->requestTemperatures();
  }

  delay(750); // conversion time @ 12-bit

  for (uint8_t i = 0; i < sensorCount; i++) {
    SensorInfo& s = sensors[i];
    float t = dallas[s.bus]->getTempC(s.rom);
    s.temperatureC = t;

    Serial.printf(
      "Bus %u | ROM=%s | GID=%llu | Temp=%.2f °C\n",
      s.bus,
      romToHex(s.rom).c_str(),
      romToGrentonId(s.rom),
      t
    );
  }

  Serial.println("----------------------------------------");
}

// ---------- Setup / Loop ----------

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\nESP32 1-Wire Multi-Bus Test");

  for (uint8_t b = 0; b < BUS_COUNT; b++) {
    oneWire[b] = new OneWire(BUS_PINS[b]);
    dallas[b] = new DallasTemperature(oneWire[b]);
    dallas[b]->begin();
    dallas[b]->setWaitForConversion(false); // async-safe
  }

  enumerateAllBuses();
}

void loop() {
  pollSensors();
  delay(POLL_INTERVAL_MS);
}
