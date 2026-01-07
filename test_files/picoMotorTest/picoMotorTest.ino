// Motor A (Front Left)
#define AIN1 2
#define AIN2 3
#define PWMA 4

// Motor B (Front Right)
#define BIN1 5
#define BIN2 6
#define PWMB 7

// Motor C (Rear Left)
#define CIN1 10
#define CIN2 11
#define PWMC 12

// Motor D (Rear Right)
#define DIN1 13
#define DIN2 14
#define PWMD 15

#define STBY 8
#define LED_BUILTIN 25

int speedVal = 255;

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(CIN1, OUTPUT); pinMode(CIN2, OUTPUT); pinMode(PWMC, OUTPUT);
  pinMode(DIN1, OUTPUT); pinMode(DIN2, OUTPUT); pinMode(PWMD, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.println("Motor Test Starting in 2s...");
  delay(2000);
}

void loop() {
  Serial.println("Spinning all motors FORWARD");
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Forward
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, speedVal);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, speedVal);
  digitalWrite(CIN1, HIGH); digitalWrite(CIN2, LOW); analogWrite(PWMC, speedVal);
  digitalWrite(DIN1, HIGH); digitalWrite(DIN2, LOW); analogWrite(PWMD, speedVal);

  delay(2000);

  Serial.println("Spinning all motors BACKWARD");
  digitalWrite(LED_BUILTIN, LOW);
  
  // Backward
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, speedVal);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, speedVal);
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, HIGH); analogWrite(PWMC, speedVal);
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, HIGH); analogWrite(PWMD, speedVal);

  delay(2000);

  Serial.println("Stopping all motors");
  
  // Stop all
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
  digitalWrite(CIN1, LOW); digitalWrite(CIN2, LOW); analogWrite(PWMC, 0);
  digitalWrite(DIN1, LOW); digitalWrite(DIN2, LOW); analogWrite(PWMD, 0);

  delay(3000); // Wait before next cycle
}
