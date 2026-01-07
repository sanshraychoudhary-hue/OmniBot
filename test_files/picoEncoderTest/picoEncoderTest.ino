volatile long ticksFL_A = 0;

void encFL_A() {
  ticksFL_A++;
}

void setup() {
  Serial.begin(115200);
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), encFL_A, RISING);
}

void loop() {
  Serial.println(ticksFL_A);
  delay(500);
}
