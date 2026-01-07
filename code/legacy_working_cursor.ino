/**********************************************************
 *                    PIN CONNECTIONS                     *
 **********************************************************

 Wi-Fi Control Bot with 4 Motors & 8 Encoder Inputs
 Controller: Raspberry Pi Pico W
 Motor Drivers: 2 x TB6612FNG (4 motors total)

 ------------------- MOTOR CONNECTIONS --------------------

 Motor A (Front Left):
   AIN1  -> GP2
   AIN2  -> GP3
   PWMA  -> GP4

 Motor B (Front Right):
   BIN1  -> GP5
   BIN2  -> GP6
   PWMB  -> GP7

 Motor C (Rear Left):
   CIN1  -> GP10
   CIN2  -> GP11
   PWMC  -> GP12

 Motor D (Rear Right):
   DIN1  -> GP13
   DIN2  -> GP14
   PWMD  -> GP15

 STBY (Standby for both drivers): 
   STBY  -> GP8

 ------------------- ENCODER CONNECTIONS ------------------

 Front Left Encoder (Motor A):
   Channel A -> GP16 (FL_ENC_A)
   Channel B -> GP17 (FL_ENC_B)

 Front Right Encoder (Motor B):
   Channel A -> GP18 (FR_ENC_A)
   Channel B -> GP19 (FR_ENC_B)

 Rear Left Encoder (Motor C):
   Channel A -> GP20 (RL_ENC_A)
   Channel B -> GP21 (RL_ENC_B)

 Rear Right Encoder (Motor D):
   Channel A -> GP22 (RR_ENC_A)
   Channel B -> GP26 (RR_ENC_B)

------------------- WHEEL SPECS ------------------
DIA = 1.5 inch
Ticks per revoultion = 520

------------------- PCB/WIRING INFO ------------------
A-B A-B A-B A-B PWM-B-A PWM-B-A PWM-B-A PWM-B-A

-------------------SERVO INFO ------------------
Base: GP9
Left Hand: GP8
Right Hand: GP27
Gripper: GP28

**********************************************************/

#include <WiFi.h>

#include <Servo.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>


// Wi-Fi credentials structure
struct WiFiNetwork {
  const char* ssid;
  const char* password;
};

// List of known networks
WiFiNetwork knownNetworks[] = {
  {"SurPoon", "gogetsomehelpfromgod1234"},
  {"IDEA LAB_2.4G", "ggsipu@321"}
  };

const int numNetworks = sizeof(knownNetworks) / sizeof(knownNetworks[0]);


// Wi-Fi credentials
//const char* ssid = "SurPoon";
//const char* password = "gogetsomehelpfromgod1234";

// OLED Display for SH1106
#define OLED_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define OLED_RESET -1
//#define OLED_SDA 16
//#define OLED_SCL 17
Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET);

// -- Heartbeat Globals --
unsigned long lastHeartbeatMillis = 0;
bool heartbeatState = false;

// Wi-Fi Credentials
//const char* ssid = "IDEA LAB_2.4G";
//const char* password = "ggsipu@321";

// Motor A (Front Left)
#define AIN1 2
#define AIN2 3
#define PWMA 4

// Motor B (Front Right)
#define BIN1 6  // changed from 6 to 0
#define BIN2 5
#define PWMB 7  // changed from 7 to 1

// Motor C (Rear Left)--- swapped for direction change
#define CIN1 10
#define CIN2 11
#define PWMC 12

// Motor D (Rear Right)--- swapped for direction change
#define DIN1 14
#define DIN2 13
#define PWMD 15

//#define STBY 8
//#define LED_PIN 25



// --------- Encoder Pin Definitions ---------
#define FL_ENC_A 0 // 16
#define FL_ENC_B 1 // 17
#define FR_ENC_A 19
#define FR_ENC_B 18
#define RL_ENC_A 22
#define RL_ENC_B 26
#define RR_ENC_A 21
#define RR_ENC_B 20

// --------- Encoder Tick Counters (Signed) ---------
volatile long ticksFL = 0;
volatile long ticksFR = 0;
volatile long ticksRL = 0;
volatile long ticksRR = 0;

// --- Per-Wheel PID & Movement Control ---
struct WheelPID {
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
  float integral = 0;
  float last_error = 0;
  long start_ticks = 0;
  long target_ticks = 0;
  bool active = false;
};

WheelPID pidFL, pidFR, pidRL, pidRR;
bool move_active = false;

//Wheel-Dimensions Globals
float wheel_diameter_cm = 3.81;  // 1.5 inch in cm
int encoder_ticks_per_rev = 520;
float total_distance_cm = 0;
int last_ticks_sum = 0;
int current_angle = 0;


// --------- Encoder ISRs with Direction Detection ---------
void encFL_ISR() {
  if (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B))
    ticksFL++;
  else
    ticksFL--;
}

void encFR_ISR() {
  if (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B))
    ticksFR++;
  else
    ticksFR--;
}

void encRL_ISR() {
  if (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B))
    ticksRL++;
  else
    ticksRL--;
}

void encRR_ISR() {
  if (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B))
    ticksRR++;
  else
    ticksRR--;
}


WiFiServer server(80);
int speedVal = 200;
String currentState = "Stopped";

Servo servoBase, servoLeft, servoRight, servoGrip;

void setup() {
  Serial.begin(115200);

  // OLED Display Setup
  //Wire.setSDA(OLED_SDA);
  //Wire.setSCL(OLED_SCL);
  //Wire.begin();
  
  if(!display.begin(OLED_I2C_ADDRESS, true)) {
    Serial.println(F("SH1106 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 10);
    display.println("Searching...");
    display.display();
  }

  // Setup motor control pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(CIN1, OUTPUT); pinMode(CIN2, OUTPUT); pinMode(PWMC, OUTPUT);
  pinMode(DIN1, OUTPUT); pinMode(DIN2, OUTPUT); pinMode(PWMD, OUTPUT);
//  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

 // Encoder Pins as Input
pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

// Attach interrupts to Channel A of each encoder
attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encFL_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encFR_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(RL_ENC_A), encRL_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(RR_ENC_A), encRR_ISR, RISING);

  // Attach servos to their GPIO pins
  servoBase.attach(9);
  servoLeft.attach(8);
  servoRight.attach(27);
  servoGrip.attach(28);

  // --- Smart WiFi Connection ---
  bool connected = false;
  for (int i = 0; i < numNetworks; i++) {
    // Update display to show current attempt
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("Trying:");
    display.setTextSize(2);
    display.setCursor(0, 25);
    display.println(knownNetworks[i].ssid);
    display.display();
    
    WiFi.begin(knownNetworks[i].ssid, knownNetworks[i].password);
    Serial.print("\nAttempting to connect to ");
    Serial.println(knownNetworks[i].ssid);

    // Wait for connection with a 7-second timeout
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 7000) {
      delay(500);
      Serial.print(".");
    }

    // If connected, break the loop
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    } else {
      Serial.println("\nFailed to connect.");
      WiFi.disconnect(true); // Disconnect and try next network
      delay(100);
    }
  }

  // Final status update on display
  display.clearDisplay();
  display.setCursor(0,0);
  if (connected) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    
    display.setTextSize(2);
    display.println("Connected!");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.setCursor(0, 32);
    display.print("SSID: ");
    display.println(WiFi.SSID());
  } else {
    Serial.println("\nCould not connect to any WiFi network.");
    display.setTextSize(2);
    display.println("Connection");
    display.println("Failed");
  }
  display.display();


  // Connect to Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500); Serial.print(".");
  //   display.print(".");
  //   display.display();
  // }

  // Serial.println("\nWiFi connected.");
  // Serial.print("IP address: "); Serial.println(WiFi.localIP());

  // // Update OLED
  // display.clearDisplay();
  // display.setCursor(0,0);
  // display.setTextSize(2);
  // display.println("Connected!");
  // display.setTextSize(1);
  // display.setCursor(0, 20);
  // display.print("IP Address:");
  // display.setCursor(0, 30);
  // display.print(WiFi.localIP());
  // display.display();

  server.begin();
}

void loop() {
  // --- OLED Heartbeat ---
  // Toggles a pixel in the top-right corner to show the Pico is running.
  if (millis() - lastHeartbeatMillis > 500) {
    lastHeartbeatMillis = millis();
    heartbeatState = !heartbeatState;
    display.fillRect(DISPLAY_WIDTH - 3, 0, 3, 3, heartbeatState ? SH110X_WHITE : SH110X_BLACK);
    display.display();
  }
  
  WiFiClient client = server.available();
  updateDistanceMoved();

  if (move_active) {
    updateAllWheelPIDs();
}

  if (client) {
    Serial.println("\nClient connected");

    String request = client.readStringUntil('\r');
    client.flush();
    Serial.print("Request: "); Serial.println(request);

    if (request.indexOf("/servo?") != -1) {
      if (request.indexOf("base=") != -1) {
        int val = request.substring(request.indexOf("base=") + 5).toInt();
        servoBase.write(constrain(val, 0, 180));
      } else if (request.indexOf("left=") != -1) {
        int val = request.substring(request.indexOf("left=") + 5).toInt();
        servoLeft.write(constrain(val, 0, 120));
      } else if (request.indexOf("right=") != -1) {
        int val = request.substring(request.indexOf("right=") + 6).toInt();
        servoRight.write(constrain(val, 70, 180));
      } else if (request.indexOf("grip=") != -1) {
        int val = request.substring(request.indexOf("grip=") + 5).toInt();
        servoGrip.write(constrain(val, 0, 120));
      }
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.print("OK");
      return;
    }
    else if (request.indexOf("/move_rt?") != -1) {
      float dist = request.substring(request.indexOf("dist=") + 5, request.indexOf("&")).toFloat();
      float angle = request.substring(request.indexOf("angle=") + 6).toFloat();
      startMoveRT(dist, angle);
    }
    else if (request.indexOf("/move_xy?") != -1) {
      float x = request.substring(request.indexOf("x=") + 2, request.indexOf("&")).toFloat();
      float y = request.substring(request.indexOf("y=") + 2).toFloat();
      startMoveXY(x, y);
    }
    else if (request.indexOf("/stop_all_move") != -1) {
      stopAllMove();
    }
    else if (request.indexOf("/set_pid?") != -1) {
       // Example: /set_pid?wheel=fl&p=1.2&i=0.1&d=0.3
      String wheel = request.substring(request.indexOf("wheel=") + 6, request.indexOf("&"));
      float p = request.substring(request.indexOf("p=") + 2, request.indexOf("&", request.indexOf("p="))).toFloat();
      float i = request.substring(request.indexOf("i=") + 2, request.indexOf("&", request.indexOf("i="))).toFloat();
      float d = request.substring(request.indexOf("d=") + 2).toFloat();
      
      if (wheel == "fl") { pidFL.Kp = p; pidFL.Ki = i; pidFL.Kd = d; }
      else if (wheel == "fr") { pidFR.Kp = p; pidFR.Ki = i; pidFR.Kd = d; }
      else if (wheel == "rl") { pidRL.Kp = p; pidRL.Ki = i; pidRL.Kd = d; }
      else if (wheel == "rr") { pidRR.Kp = p; pidRR.Ki = i; pidRR.Kd = d; }
    }
    else if (request.indexOf("/vector") != -1) {
      int angleStart = request.indexOf("angle=") + 6;
int forceStart = request.indexOf("force=") + 6;
int ampIndex = request.indexOf('&');

int angle = 0;
float force = 0;

if (angleStart > 5 && forceStart > 5) {
  if (angleStart < forceStart) {
    angle = request.substring(angleStart, ampIndex).toInt();
    force = request.substring(forceStart).toFloat();
  } else {
    force = request.substring(forceStart, ampIndex).toFloat();
    angle = request.substring(angleStart).toInt();
  }
}


      int dynamicSpeed = int(constrain(force * 255, 0, 255));
      handleAnalogDirection(angle, dynamicSpeed);
    }
    else if (request.indexOf("/FL") != -1) moveForwardLeft(speedVal);
    else if (request.indexOf("/FR") != -1) moveForwardRight(speedVal);
    else if (request.indexOf("/F") != -1) moveForward(speedVal);
    else if (request.indexOf("/BL") != -1) moveBackwardLeft(speedVal);
    else if (request.indexOf("/BR") != -1) moveBackwardRight(speedVal);
    else if (request.indexOf("/CW") != -1) rotateCW(speedVal);
    else if (request.indexOf("/CCW") != -1) rotateCCW(speedVal);
    else if (request.indexOf("/B") != -1) moveBackward(speedVal);
    else if (request.indexOf("/L") != -1) moveLeft(speedVal);
    else if (request.indexOf("/R") != -1) moveRight(speedVal);
    else if (request.indexOf("/S") != -1) stopBot();
    else if (request.indexOf("/reset_encoders") != -1) {
      ticksFL = 0;
      ticksFR = 0;
      ticksRL = 0;
      ticksRR = 0;
      Serial.println("Encoders reset.");
    }

    else if (request.indexOf("/speed=") != -1) {
      int val = request.substring(request.indexOf('=') + 1).toInt();
      speedVal = constrain(val, 0, 255);
      Serial.print("Speed set to: "); Serial.println(speedVal);
    }
    else if (request.indexOf("/reset_distance") != -1) {
      total_distance_cm = 0;
      last_ticks_sum = abs(ticksFL) + abs(ticksFR) + abs(ticksRL) + abs(ticksRR);
      Serial.println("Distance reset.");
    }
    else if (request.indexOf("/status") != -1) {
      
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println();
      client.print("{"
  "\"state\":\"" + currentState + "\","
  "\"speed\":" + String(speedVal) + ","
  "\"fl\":" + String(ticksFL) + ","
  "\"fr\":" + String(ticksFR) + ","
  "\"rl\":" + String(ticksRL) + ","
  "\"rr\":" + String(ticksRR) + ","
  "\"angle\":" + String(current_angle) + ","
  "\"distance\":" + String(total_distance_cm, 2) +
"}");


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

// Move Forward: Bot moves straight ahead ↑
// A (FL), B (FR), C (RL), D (RR) => All Forward
void moveForward(int spd) {
  currentState = "Moving Forward";
  Serial.println(currentState);

  // Motor A (Front Left)
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, spd);

  // Motor B (Front Right)
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, spd);

  // Motor C (Rear Left)
  digitalWrite(CIN1, HIGH); digitalWrite(CIN2, LOW); analogWrite(PWMC, spd);

  // Motor D (Rear Right)
  digitalWrite(DIN1, HIGH); digitalWrite(DIN2, LOW); analogWrite(PWMD, spd);

  blinkLED();
}

// Move Backward: Bot moves straight backward ↓
// A, B, C, D => All Backward
void moveBackward(int spd) {
  currentState = "Moving Backward";
  Serial.println(currentState);

  // Motor A (Front Left)
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spd);

  // Motor B (Front Right)
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spd);

  // Motor C (Rear Left)
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, HIGH); analogWrite(PWMC, spd);

  // Motor D (Rear Right)
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, HIGH); analogWrite(PWMD, spd);

  blinkLED();
}

// Move Left: Bot strafes left ←
// A (FL) & D (RR) => Backward
// B (FR) & C (RL) => Forward
void moveLeft(int spd) {
  currentState = "Moving Left";
  Serial.println(currentState);

  // Motor A (Front Left)
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spd);

  // Motor B (Front Right)
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, spd);

  // Motor C (Rear Left)
  digitalWrite(CIN1, HIGH); digitalWrite(CIN2, LOW); analogWrite(PWMC, spd);

  // Motor D (Rear Right)
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, HIGH); analogWrite(PWMD, spd);

  blinkLED();
}

// Move Right: Bot strafes right →
// A (FL) & D (RR) => Forward
// B (FR) & C (RL) => Backward
void moveRight(int spd) {
  currentState = "Moving Right";
  Serial.println(currentState);

  // Motor A (Front Left)
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, spd);

  // Motor B (Front Right)
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spd);

  // Motor C (Rear Left)
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, HIGH); analogWrite(PWMC, spd);

  // Motor D (Rear Right)
  digitalWrite(DIN1, HIGH); digitalWrite(DIN2, LOW); analogWrite(PWMD, spd);

  blinkLED();
}

// Rotate Clockwise: Left motors forward, right motors backward ↻
void rotateCW(int spd) {
  currentState = "Rotating Clockwise";
  Serial.println(currentState);

  // Motor A (FL) forward
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, spd);

  // Motor B (FR) backward
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spd);

  // Motor C (RL) forward
  digitalWrite(CIN1, HIGH); digitalWrite(CIN2, LOW); analogWrite(PWMC, spd);

  // Motor D (RR) backward
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, HIGH); analogWrite(PWMD, spd);

  blinkLED();
}

// Rotate Counterclockwise: Left motors backward, right motors forward ↺
void rotateCCW(int spd) {
  currentState = "Rotating Counterclockwise";
  Serial.println(currentState);

  // Motor A (FL) backward
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spd);

  // Motor B (FR) forward
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, spd);

  // Motor C (RL) backward
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, HIGH); analogWrite(PWMC, spd);

  // Motor D (RR) forward
  digitalWrite(DIN1, HIGH); digitalWrite(DIN2, LOW); analogWrite(PWMD, spd);

  blinkLED();
}

// Move Forward Left (↖): A & C forward, others off
// Move Forward Left (↖): A (FL) backward, B (FR) forward, C (RL) forward, D (RR) off
void moveForwardLeft(int spd) {
  currentState = "Moving Forward Left";
  Serial.println(currentState);

  // Motor A (FL) BACKWARD
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, spd);

  // Motor B (FR) FORWARD
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, spd);

  // Motor C (RL) FORWARD
  digitalWrite(CIN1, HIGH); digitalWrite(CIN2, LOW); analogWrite(PWMC, spd);

  // Motor D (RR) OFF
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, LOW); analogWrite(PWMD, 0);

  blinkLED();
}


// Move Forward Right (↗): A (FL) forward, B (FR) backward, D (RR) forward, C (RL) off
void moveForwardRight(int spd) {
  currentState = "Moving Forward Right";
  Serial.println(currentState);

  // Motor A (FL) FORWARD
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, spd);

  // Motor B (FR) BACKWARD
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, spd);

  // Motor D (RR) FORWARD
  digitalWrite(DIN1, HIGH); digitalWrite(DIN2, LOW); analogWrite(PWMD, spd);

  // Motor C (RL) OFF
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, LOW); analogWrite(PWMC, 0);

  blinkLED();
}


// Move Backward Left (↙): A & D backward, others off
void moveBackwardLeft(int spd) {
  currentState = "Moving Backward Left";
  Serial.println(currentState);

  // Motor A (FL)
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spd);

  // Motor D (RR)
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, HIGH); analogWrite(PWMD, spd);

  // B & C off
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, LOW); analogWrite(PWMC, 0);

  blinkLED();
}

// Move Backward Right (↘): B & C backward, others off
void moveBackwardRight(int spd) {
  currentState = "Moving Backward Right";
  Serial.println(currentState);

  // Motor B (FR)
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spd);

  // Motor C (RL)
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, HIGH); analogWrite(PWMC, spd);

  // A & D off
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, LOW); analogWrite(PWMD, 0);

  blinkLED();
}


// Stop: Disable all motors
void stopBot() {
  currentState = "Stopped";
  Serial.println(currentState);
  
  // Front Left Motor
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  // Front Right Motor
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);

  // Rear Left Motor
  digitalWrite(CIN1, LOW);
  digitalWrite(CIN2, LOW);
  analogWrite(PWMC, 0);

  // Rear Right Motor
  digitalWrite(DIN1, LOW);
  digitalWrite(DIN2, LOW);
  analogWrite(PWMD, 0);
}

void handleAnalogDirection(int angle, int spd) {
  currentState = "Analog Joystick Movement";

  if (angle >= 337 || angle < 22) {
    moveForward(spd);
  } else if (angle >= 22 && angle < 67) {
    moveForwardRight(spd);
  } else if (angle >= 67 && angle < 112) {
    moveRight(spd);
  } else if (angle >= 112 && angle < 157) {
    moveBackwardRight(spd);
  } else if (angle >= 157 && angle < 202) {
    moveBackward(spd);
  } else if (angle >= 202 && angle < 247) {
    moveBackwardLeft(spd);
  } else if (angle >= 247 && angle < 292) {
    moveLeft(spd);
  } else if (angle >= 292 && angle < 337) {
    moveForwardLeft(spd);
  }
  current_angle = angle;
}

void updateDistanceMoved() {
  int total_ticks = abs(ticksFL) + abs(ticksFR) + abs(ticksRL) + abs(ticksRR);
  int delta_ticks = total_ticks - last_ticks_sum;

  float k = 0.65 + (0.90 - 0.65) * abs(cos(2 * current_angle * PI / 180.0)); // k(θ)
  float cm_per_tick = (PI * wheel_diameter_cm / encoder_ticks_per_rev) * k;
  total_distance_cm += (delta_ticks / 4.0) * cm_per_tick;

  last_ticks_sum = total_ticks;
}
  
// --- New Per-Wheel PID Control Functions ---

void calcOmniTicks(float distance_cm, float move_angle_deg, long (&ticks)[4]) {
  float wheel_angles_deg[4] = {45, 135, 225, 315};
  float wheel_circ = PI * wheel_diameter_cm;

  for (int i = 0; i < 4; i++) {
    float move_angle_rad = move_angle_deg * PI / 180.0;
    float theta_rad = wheel_angles_deg[i] * PI / 180.0;
    float cos_term = cos(move_angle_rad - theta_rad);

    if (abs(cos_term) < 1e-4) {
      ticks[i] = 0;
    } else {
      float eff_dist = distance_cm / abs(cos_term);
      ticks[i] = round(eff_dist / wheel_circ * encoder_ticks_per_rev);
    }
  }
}

void startMoveRT(float distance, float angle) {
  long targetTicks[4];
  calcOmniTicks(distance, angle, targetTicks);

  pidFL = {pidFL.Kp, pidFL.Ki, pidFL.Kd, 0, 0, ticksFL, targetTicks[0], true};
  pidFR = {pidFR.Kp, pidFR.Ki, pidFR.Kd, 0, 0, ticksFR, targetTicks[1], true};
  pidRL = {pidRL.Kp, pidRL.Ki, pidRL.Kd, 0, 0, ticksRL, targetTicks[2], true};
  pidRR = {pidRR.Kp, pidRR.Ki, pidRR.Kd, 0, 0, ticksRR, targetTicks[3], true};
  
  currentState = "Auto Movement";
  move_active = true;
}

void startMoveXY(float x, float y) {
  float distance = sqrt(x * x + y * y);
  float angle = atan2(y, x) * 180.0 / PI;
  if (angle < 0) {
    angle += 360;
  }
  startMoveRT(distance, angle);
}

void stopAllMove() {
  // Deactivate all PID controllers
  pidFL.active = false;
  pidFR.active = false;
  pidRL.active = false;
  pidRR.active = false;
  move_active = false;
  
  // Stop all motors immediately
  stopBot();

  // Reset all encoder ticks to zero
  ticksFL = 0;
  ticksFR = 0;
  ticksRL = 0;
  ticksRR = 0;

  // Reset distance calculation variables
  total_distance_cm = 0;
  last_ticks_sum = 0;

  Serial.println("AUTONOMOUS MOVE STOPPED AND ALL COUNTERS RESET.");
}

void updateWheelPID(WheelPID& pid, volatile long& current_ticks, int pwm_pin, int in1, int in2) {
  if (!pid.active) return;

  long current_error = pid.target_ticks - (current_ticks - pid.start_ticks);
  pid.integral += current_error;
  
  // Basic anti-windup for integral term to prevent it from growing too large
  if (abs(pid.integral * pid.Ki) > 255) {
    pid.integral -= current_error;
  }

  float derivative = current_error - pid.last_error;
  
  float output = pid.Kp * current_error + pid.Ki * pid.integral + pid.Kd * derivative;
  pid.last_error = current_error;

  output = constrain(output, -255, 255);

  // Add a small deadband to prevent motor whine/jitter when the target is held perfectly
  if (abs(current_error) <= 2) {
    analogWrite(pwm_pin, 0);
    return;
  }

  long final_pwm = 0;
  if (output > 0) {
    // Map the PID output to the effective PWM range (70-255)
    final_pwm = map(long(output), 1, 255, 70, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else { // output is negative
    final_pwm = map(long(abs(output)), 1, 255, 70, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(pwm_pin, final_pwm);
}

void updateAllWheelPIDs() {
  updateWheelPID(pidFL, ticksFL, PWMA, AIN1, AIN2);
  updateWheelPID(pidFR, ticksFR, PWMB, BIN1, BIN2);
  updateWheelPID(pidRL, ticksRR, PWMD, DIN1, DIN2); // Corrected: pidRL uses Motor D, gets feedback from ticksRR
  updateWheelPID(pidRR, ticksRL, PWMC, CIN1, CIN2); // Corrected: pidRR uses Motor C, gets feedback from ticksRL

  // The check for move completion has been removed.
  // The PID will remain active, holding the position, until 'stopAllMove' is called.
}

void sendWebPage(WiFiClient& client) {
  client.println(R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>OmniServe Bot Dashboard</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f9f9f9;
      text-align: center;
      margin: 0;
      padding: 0;
    }

    h2 {
      margin-top: 20px;
    }

    .btn {
      width: 70px;
      height: 50px;
      font-size: 24px;
      margin: 8px;
      border-radius: 8px;
      border: 2px solid #333;
      background-color: #fff;
      cursor: pointer;
      transition: 0.2s;
    }

    .btn:hover {
      background-color: #d0e7ff;
    }

    .control-grid {
      display: inline-block;
      margin-top: 10px;
    }

    .status {
      font-weight: bold;
      font-size: 18px;
      margin: 15px;
    }

    #speedVal {
      font-weight: bold;
    }

    #speedContainer {
      margin: 10px auto 20px;
    }

    #speedSlider {
      width: 60%;
      margin-top: 5px;
    }

    .rotation-buttons {
      margin-top: 12px;
    }

    #joystick {
      width: 240px;
      height: 240px;
      margin: 40px auto 80px;
      position: relative;
    }

    .telemetry {
      font-weight: bold;
      font-size: 16px;
      margin: 10px;
    }
    
    #resetBtn {
  font-size: 14px;
  padding: 6px 12px;
}

  </style>
</head>
<body>
  <h2>OmniServe Bot Dashboard</h2>

  <!-- Status -->
  <p class="status" id="status">Status: Loading...</p>

  <!-- Encoder Display with Reset Encoders (left) and Reset Distance (right) -->
  <div style="display: flex; justify-content: center; align-items: center; gap: 10px; margin-bottom: 0;">
    <button class="btn" id="resetEncBtn" style="font-size: 14px; padding: 6px 12px; margin-right: 10px; height: 40px; align-self: flex-start;" onclick="resetEncoders()">Reset Encoders</button>
    <p class="status" id="encoders" style="margin-bottom: 0;">
      FL: <span id="fl">0</span> |
      FR: <span id="fr">0</span><br />
      RL: <span id="rl">0</span> |
      RR: <span id="rr">0</span>
    </p>
  </div>

  <!-- Direction, PWM & Distance (with Reset Distance button on the left) -->
  <div class="telemetry" style="display: flex; align-items: center; justify-content: center; gap: 10px; margin-bottom: 10px;">
    <button class="btn" id="resetBtn" style="font-size: 14px; padding: 6px 12px;" onclick="resetDistance()">Reset Distance</button>
    <span>Joystick Angle: <span id="angleVal">0</span>°</span> |
    <span>PWM: <span id="pwmVal">0</span></span> |
    <span>Distance: <span id="distanceVal">0.00</span> cm</span>
  </div>

  <!-- Speed Control -->
  <div id="speedContainer">
    <h4>Speed: <span id="speedVal">200</span></h4>
    <input type="range" min="0" max="255" value="200" id="speedSlider" onchange="setSpeed(this.value)" />
  </div>


  <!-- Movement Panels -->
<div style="display: flex; justify-content: center; gap: 20px; margin-top: 10px;">
  <!-- r, theta panel -->
  <div class="telemetry" style="border: 1px solid #aaa; border-radius: 8px; padding: 12px;">
    <h4>Move by r, θ</h4>
    Distance (cm): <input id="rIn" type="number" value="10" style="width:60px"> <br>
    Angle (°): <input id="thetaIn" type="number" value="0" style="width:60px"> <br>
    <button class="btn" style="font-size: 14px; padding: 6px 12px;" onclick="startMoveRT()">Start</button>
    <button class="btn" style="font-size: 14px; padding: 6px 12px;" onclick="stopAllMove()">Stop</button>
  </div>
  <!-- x, y panel -->
  <div class="telemetry" style="border: 1px solid #aaa; border-radius: 8px; padding: 12px;">
    <h4>Move to (x, y)</h4>
    X (cm): <input id="xIn" type="number" value="10" style="width:60px"> <br>
    Y (cm): <input id="yIn" type="number" value="0" style="width:60px"> <br>
    <button class="btn" style="font-size: 14px; padding: 6px 12px;" onclick="startMoveXY()">Start</button>
    <button class="btn" style="font-size: 14px; padding: 6px 12px;" onclick="stopAllMove()">Stop</button>
  </div>
</div>

<!-- Per-wheel PID panel -->
<div class="telemetry" style="margin-top: 20px; border: 1px solid #aaa; border-radius: 8px; padding: 12px; display: inline-block;">
  <h4>Per-Wheel PID Tuning</h4>
  <table style="text-align: center;">
    <thead><tr><th>Wheel</th><th>Kp</th><th>Ki</th><th>Kd</th><th></th></tr></thead>
    <tbody>
      <tr><td>FL</td><td><input id="kp_fl" type="number" value="1.0" step="0.1" style="width:50px"></td><td><input id="ki_fl" type="number" value="0.0" step="0.1" style="width:50px"></td><td><input id="kd_fl" type="number" value="0.0" step="0.1" style="width:50px"></td><td><button style="font-size:12px; padding: 2px 5px;" onclick="setPID('fl')">Set</button></td></tr>
      <tr><td>FR</td><td><input id="kp_fr" type="number" value="1.0" step="0.1" style="width:50px"></td><td><input id="ki_fr" type="number" value="0.0" step="0.1" style="width:50px"></td><td><input id="kd_fr" type="number" value="0.0" step="0.1" style="width:50px"></td><td><button style="font-size:12px; padding: 2px 5px;" onclick="setPID('fr')">Set</button></td></tr>
      <tr><td>RL</td><td><input id="kp_rl" type="number" value="1.0" step="0.1" style="width:50px"></td><td><input id="ki_rl" type="number" value="0.0" step="0.1" style="width:50px"></td><td><input id="kd_rl" type="number" value="0.0" step="0.1" style="width:50px"></td><td><button style="font-size:12px; padding: 2px 5px;" onclick="setPID('rl')">Set</button></td></tr>
      <tr><td>RR</td><td><input id="kp_rr" type="number" value="1.0" step="0.1" style="width:50px"></td><td><input id="ki_rr" type="number" value="0.0" step="0.1" style="width:50px"></td><td><input id="kd_rr" type="number" value="0.0" step="0.1" style="width:50px"></td><td><button style="font-size:12px; padding: 2px 5px;" onclick="setPID('rr')">Set</button></td></tr>
    </tbody>
  </table>
</div>


<!-- Servo Control Panel -->
<div class="dashboard-row" style="width:100%; justify-content: center; margin-top: 15px;">
  <div class="panel-box" style="min-width:320px; max-width:420px; width:100%;">
    <h4>Servo Control</h4>
    <div style="margin-bottom:10px;">
      <label for="baseSlider">Base: <span id="baseVal">90</span>°</label><br>
      <input type="range" min="0" max="180" value="90" id="baseSlider" oninput="baseVal.innerText=this.value; setServo('base',this.value)">
    </div>
    <div style="margin-bottom:10px;">
      <label for="leftSlider">Left Hand: <span id="leftVal">60</span>°</label><br>
      <input type="range" min="0" max="120" value="60" id="leftSlider" oninput="leftVal.innerText=this.value; setServo('left',this.value)">
    </div>
    <div style="margin-bottom:10px;">
      <label for="rightSlider">Right Hand: <span id="rightVal">120</span>°</label><br>
      <input type="range" min="70" max="180" value="120" id="rightSlider" oninput="rightVal.innerText=this.value; setServo('right',this.value)">
    </div>
    <div style="margin-bottom:10px;">
      <label for="gripSlider">Gripper: <span id="gripVal">60</span>°</label><br>
      <input type="range" min="0" max="120" value="60" id="gripSlider" oninput="gripVal.innerText=this.value; setServo('grip',this.value)">
    </div>
  </div>
</div>

  <!-- D-Pad 3x3 -->
  <div class="control-grid">
    <div>
      <button class="btn" id="btnFL">↖</button>
      <button class="btn" id="btnF">↑</button>
      <button class="btn" id="btnFR">↗</button>
    </div>
    <div>
      <button class="btn" id="btnL">←</button>
      <button class="btn" id="btnS">●</button>
      <button class="btn" id="btnR">→</button>
    </div>
    <div>
      <button class="btn" id="btnBL">↙</button>
      <button class="btn" id="btnB">↓</button>
      <button class="btn" id="btnBR">↘</button>
    </div>
  </div>

  <!-- Rotation Buttons -->
  <div class="rotation-buttons">
    <button class="btn" id="btnCCW">⟲</button>
    <button class="btn" id="btnCW">⟳</button>
  </div>

  <!-- Joystick -->
  <div id="joystick"></div>

  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
  <script>

    function resetDistance() {
  fetch('/reset_distance')
    .then(() => {
      document.getElementById("distanceVal").innerText = "0.00";
    });
}
    function resetEncoders() {
      fetch('/reset_encoders')
        .then(() => {
          document.getElementById("fl").innerText = 0;
          document.getElementById("fr").innerText = 0;
          document.getElementById("rl").innerText = 0;
          document.getElementById("rr").innerText = 0;
        });
    }
    function sendCmd(cmd) {
      fetch('/' + cmd);
      fetchStatus();
    }

    function setSpeed(val) {
      document.getElementById('speedVal').innerText = val;
      fetch('/speed=' + val);
    }

    function startMoveRT() {
      const dist = document.getElementById("rIn").value;
      const angle = document.getElementById("thetaIn").value;
      fetch(`/move_rt?dist=${dist}&angle=${angle}`);
    }

    function startMoveXY() {
      const x = document.getElementById("xIn").value;
      const y = document.getElementById("yIn").value;
      fetch(`/move_xy?x=${x}&y=${y}`);
    }

    function stopAllMove() {
      fetch('/stop_all_move');
    }

    function setPID(wheel) {
      const p = document.getElementById(`kp_${wheel}`).value;
      const i = document.getElementById(`ki_${wheel}`).value;
      const d = document.getElementById(`kd_${wheel}`).value;
      fetch(`/set_pid?wheel=${wheel}&p=${p}&i=${i}&d=${d}`);
    }

    function fetchStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          document.getElementById("status").innerText = "Status: " + data.state + " | Speed: " + data.speed;
          document.getElementById("fl").innerText = data.fl;
          document.getElementById("fr").innerText = data.fr;
          document.getElementById("rl").innerText = data.rl;
          document.getElementById("rr").innerText = data.rr;
          document.getElementById("angleVal").innerText = data.angle;
          document.getElementById("distanceVal").innerText = data.distance.toFixed(2); // This will show negative values too
        });
    }

    function bindRealtimeButton(id, cmd) {
      const btn = document.getElementById(id);
      btn.addEventListener('mousedown', () => sendCmd(cmd));
      btn.addEventListener('mouseup', () => sendCmd('S'));
      btn.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd(cmd); });
      btn.addEventListener('touchend', (e) => { e.preventDefault(); sendCmd('S'); });
    }

    const buttons = ["F", "B", "L", "R", "S", "FL", "FR", "BL", "BR", "CW", "CCW"];
    buttons.forEach(cmd => bindRealtimeButton("btn" + cmd, cmd));

    document.addEventListener('keydown', function(e) {
  const key = e.key.toLowerCase();

      // Servo control elements
      const baseSlider = document.getElementById('baseSlider');
      const leftSlider = document.getElementById('leftSlider');
      const rightSlider = document.getElementById('rightSlider');
      const gripSlider = document.getElementById('gripSlider');

      // Helper to update slider and send command
      function updateServo(slider, value) {
        slider.value = value;
        document.getElementById(slider.id.replace('Slider', 'Val')).innerText = value;
        const joint = slider.id.replace('Slider', '');
        setServo(joint, value);
      }

  switch (key) {
        // D-Pad and movement keys (existing logic)
        case 'q': sendCmd('FL'); break;
        case 'w': sendCmd('F'); break;
        case 'e': sendCmd('FR'); break;
        case 'a': sendCmd('L'); break;
        case 's': sendCmd('S'); break;
        case 'd': sendCmd('R'); break;
        case 'z': sendCmd('BL'); break;
        case 'x': sendCmd('B'); break;
        case 'c': sendCmd('BR'); break;
        case 'k': sendCmd('CW'); break;
        case 'j': sendCmd('CCW'); break;

        // Servo keyboard controls
        case '5':
          updateServo(baseSlider, Math.max(0, parseInt(baseSlider.value) - 10));
          break;
        case '6':
          updateServo(baseSlider, Math.min(180, parseInt(baseSlider.value) + 10));
          break;
        case 't':
          updateServo(leftSlider, Math.max(0, parseInt(leftSlider.value) - 10));
          break;
        case 'y':
          updateServo(leftSlider, Math.min(120, parseInt(leftSlider.value) + 10));
          break;
        case 'g':
          updateServo(rightSlider, Math.max(70, parseInt(rightSlider.value) - 10));
          break;
        case 'h':
          updateServo(rightSlider, Math.min(180, parseInt(rightSlider.value) + 10));
          break;
        case 'b': // Open gripper
          updateServo(gripSlider, 0);
          break;
        case 'n': // Close gripper
          updateServo(gripSlider, 120);
          break;
  }
});

document.addEventListener('keyup', function(e) {
  const key = e.key.toLowerCase();
  // Stop movement when any movement key is released
  if (['q','w','e','a','s','d','z','x','c','k','j'].includes(key)) {
    sendCmd('S');
  }
});


    let lastJoystickTime = Date.now();

    const joystick = nipplejs.create({
      zone: document.getElementById("joystick"),
      mode: "static",
      position: { left: '50%', top: '50%' },
      color: "blue",
      size: 300
    });

    joystick.on("move", (evt, data) => {
  if (!data || !data.angle || !data.force) return;

  // Filter out ghost signals
  if (data.force < 0.1) return;

  let angle = Math.round(data.angle.degree);
  let adjustedAngle = (angle + 270) % 360;
  let force = Math.min(Math.max(data.force, 0), 1);
  let pwm = Math.round(force * 255);

  document.getElementById("angleVal").innerText = adjustedAngle;
  document.getElementById("pwmVal").innerText = pwm;

  fetch(`/vector?angle=${adjustedAngle}&force=${force.toFixed(2)}`);
  lastJoystickTime = Date.now();
});


    joystick.on("end", () => {
      fetch("/S");
      document.getElementById("angleVal").innerText = 0;
      document.getElementById("pwmVal").innerText = 0;
    });

    // Safety timeout to stop motors
    setInterval(() => {
      if (Date.now() - lastJoystickTime > 300) {
        fetch("/S");
        document.getElementById("angleVal").innerText = 0;
        document.getElementById("pwmVal").innerText = 0;
      }
    }, 300);

    function setServo(joint, val) {
      fetch(`/servo?${joint}=${val}`);
    }

    // Bind slider inputs to update the JS state as well
    document.getElementById('baseSlider').addEventListener('input', function() { baseVal.innerText=this.value; setServo('base', this.value); });
    document.getElementById('leftSlider').addEventListener('input', function() { leftVal.innerText=this.value; setServo('left', this.value); });
    document.getElementById('rightSlider').addEventListener('input', function() { rightVal.innerText=this.value; setServo('right', this.value); });
    document.getElementById('gripSlider').addEventListener('input', function() { gripVal.innerText=this.value; setServo('grip', this.value); });

    setInterval(fetchStatus, 1000);
  </script>
</body>
</html>
)rawliteral");
}