#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("WiFi Scan with Pico W started...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Disconnect from any previous connection
  delay(100);
}

void loop() {
  Serial.println("Scanning for available networks...");

  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.printf("%d networks found:\n", n);
    for (int i = 0; i < n; ++i) {
      int rssi = WiFi.RSSI(i);
      int quality = getSignalQuality(rssi);
      int encType = WiFi.encryptionType(i);

      Serial.printf("%d: %s\n", i + 1, WiFi.SSID(i));  // No .c_str()
      Serial.printf("   Signal Strength: %d dBm (%d%%)\n", rssi, quality);
      Serial.printf("   Encryption: %s\n\n", encType == CYW43_AUTH_OPEN ? "Open" : "Encrypted");
      delay(10);
    }
  }

  delay(5000);  // Scan every 5 seconds
}

int getSignalQuality(int rssi) {
  if (rssi <= -100)
    return 0;
  else if (rssi >= -50)
    return 100;
  else
    return 2 * (rssi + 100);  // Convert dBm to percentage
}
