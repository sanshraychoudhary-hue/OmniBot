// Minimal Servo Tester for Pico W (4 servos, web UI)
#include <WiFi.h>
#include <Servo.h>

// Wi-Fi credentials
const char* ssid = "SurPoon";
const char* password = "gogetsomehelpfromgod1234";

Servo servoBase, servoLeft, servoRight, servoGrip;

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  servoBase.attach(9);    // GP9
  servoLeft.attach(8);    // GP8
  servoRight.attach(27);  // GP27
  servoGrip.attach(28);   // GP28

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();
    if (request.indexOf("/servo?") != -1) {
      blinkLED(); // Blink LED on command receipt
      Serial.print("Servo command received: ");
      Serial.println(request);

      if (request.indexOf("base=") != -1) {
        int val = request.substring(request.indexOf("base=") + 5).toInt();
        val = constrain(val, 0, 180);
        servoBase.write(val);
        Serial.print("  Base servo set to: "); Serial.println(val);
      } else if (request.indexOf("left=") != -1) {
        int val = request.substring(request.indexOf("left=") + 5).toInt();
        val = constrain(val, 0, 120);
        servoLeft.write(val);
        Serial.print("  Left servo set to: "); Serial.println(val);
      } else if (request.indexOf("right=") != -1) {
        int val = request.substring(request.indexOf("right=") + 6).toInt();
        val = constrain(val, 70, 180);
        servoRight.write(val);
        Serial.print("  Right servo set to: "); Serial.println(val);
      } else if (request.indexOf("grip=") != -1) {
        int val = request.substring(request.indexOf("grip=") + 5).toInt();
        val = constrain(val, 0, 120);
        servoGrip.write(val);
        Serial.print("  Gripper servo set to: "); Serial.println(val);
      }
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.print("OK");
      return;
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html\r\n\r\n");
    sendWebPage(client);
  }
}

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendWebPage(WiFiClient& client) {
  client.println(R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Servo Tester</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f9f9f9; text-align: center; margin: 0; padding: 0; }
    .panel { background: #fff; border-radius: 10px; box-shadow: 0 2px 12px #0001; display: inline-block; margin: 40px auto; padding: 24px 32px; }
    h2 { margin-top: 0; }
    .slider-row { margin: 18px 0; }
    label { font-size: 1.1em; }
    input[type="range"] { width: 260px; }
    .val { font-weight: bold; margin-left: 8px; }
  </style>
</head>
<body>
  <div class="panel">
    <h2>Servo Tester (Pico W)</h2>
    <div class="slider-row">
      <label for="baseSlider">Base (GP9): <span id="baseVal">90</span>°</label><br>
      <input type="range" min="0" max="180" value="90" id="baseSlider" oninput="baseVal.innerText=this.value; setServo('base',this.value)">
    </div>
    <div class="slider-row">
      <label for="leftSlider">Left Hand (GP8): <span id="leftVal">60</span>°</label><br>
      <input type="range" min="0" max="120" value="60" id="leftSlider" oninput="leftVal.innerText=this.value; setServo('left',this.value)">
    </div>
    <div class="slider-row">
      <label for="rightSlider">Right Hand (GP27): <span id="rightVal">120</span>°</label><br>
      <input type="range" min="70" max="180" value="120" id="rightSlider" oninput="rightVal.innerText=this.value; setServo('right',this.value)">
    </div>
    <div class="slider-row">
      <label for="gripSlider">Gripper (GP28): <span id="gripVal">60</span>°</label><br>
      <input type="range" min="0" max="120" value="60" id="gripSlider" oninput="gripVal.innerText=this.value; setServo('grip',this.value)">
    </div>
  </div>
  <script>
    function setServo(joint, val) {
      fetch(`/servo?${joint}=${val}`);
    }
  </script>
</body>
</html>
)rawliteral");
} 