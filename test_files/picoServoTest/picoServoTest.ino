#include <Servo.h>

// Servo objects for the robotic arm
Servo baseServo;
Servo leftServo;
Servo rightServo;
Servo gripperServo;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Attach servos to their corresponding GPIO pins on the Pico W
  baseServo.attach(9);      // Base servo to GP9
  leftServo.attach(8);      // Left hand servo to GP8
  rightServo.attach(27);    // Right hand servo to GP27
  gripperServo.attach(28);  // Gripper servo to GP28

  // Wait a moment for the serial monitor to connect
  delay(2000); 

  // Print instructions to the user
  Serial.println("--- Pico W Servo Tester ---");
  Serial.println("Enter angles in CSV format: base,left,right,gripper");
  Serial.println("Example: 90,60,120,60");
  Serial.println("\nRanges:");
  Serial.println("  Base: 0-180");
  Serial.println("  Left Hand: 0-180");
  Serial.println("  Right Hand: 0-180");
  Serial.println("  Gripper: 0-180");
  Serial.println("--------------------------------");
}

void loop() {
  // Check if there is data available to read from the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int angles[4];
    int partIndex = 0;
    int lastComma = -1;

    // Manually parse the comma-separated string
    for (int i = 0; i < input.length() && partIndex < 4; i++) {
      if (input.charAt(i) == ',') {
        angles[partIndex++] = input.substring(lastComma + 1, i).toInt();
        lastComma = i;
      }
    }
    // Get the last part after the final comma
    if (partIndex < 4) {
      angles[partIndex++] = input.substring(lastComma + 1).toInt();
    }

    // Check if we got exactly 4 angles
    if (partIndex == 4) {
      // Constrain the angles to their safe operational ranges
      int baseAngle = constrain(angles[0], 0, 180);
      int leftAngle = constrain(angles[1], 0, 180);
      int rightAngle = constrain(angles[2], 0, 180);
      int gripperAngle = constrain(angles[3], 0, 180);

      // Write the constrained angles to the servos
      baseServo.write(baseAngle);
      leftServo.write(leftAngle);
      rightServo.write(rightAngle);
      gripperServo.write(gripperAngle);

      // Print feedback to the serial monitor
      Serial.print("Moved to -> Base: ");
      Serial.print(baseAngle);
      Serial.print(", Left: ");
      Serial.print(leftAngle);
      Serial.print(", Right: ");
      Serial.print(rightAngle);
      Serial.print(", Gripper: ");
      Serial.println(gripperAngle);
    } else {
      Serial.println("Invalid input. Format: base,left,right,gripper");
    }
  }
} 
