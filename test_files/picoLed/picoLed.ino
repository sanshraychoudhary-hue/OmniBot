void setup() {
  // Initialize the built-in LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Turn LED ON
  delay(500);                       // Wait for 500 milliseconds
  digitalWrite(LED_BUILTIN, LOW);   // Turn LED OFF
  delay(500);                       // Wait for 500 milliseconds
}
