#include <WiFi.h>
#include <ETH.h>

void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  Serial.printf("ETH event: %d\n", event);
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    Serial.print("ETH GOT IP: ");
    Serial.println(ETH.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ETH test on ESP32-Stick-PoE-P");

  WiFi.onEvent(onEthEvent);

  ETH.begin(
    ETH_PHY_LAN8720,
    1,           // PHY addr = 1
    23,          // MDC
    18,          // MDIO
    -1,          // no reset pin
    ETH_CLOCK_GPIO17_OUT  // IMPORTANT: clock OUT on GPIO17
  );

  Serial.println("ETH.begin() called");
}

void loop() {
  delay(1000);
}
