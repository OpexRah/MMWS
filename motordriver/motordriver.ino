// Define pins connected to the motor driver
const int enPin = 9;   // PWM pin to control motor speed
const int in1Pin = 2;  // Control pin 1 to set motor direction
const int in2Pin = 3;  // Control pin 2 to set motor direction

void setup() {
  // Initialize all motor driver pins as outputs
  pinMode(enPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Stop the motor initially
  digitalWrite(in1Pin, LOW);  // Set control pin 1 to LOW
  digitalWrite(in2Pin, LOW);  // Set control pin 2 to LOW
  analogWrite(enPin, 0);      // Set PWM to 0, motor speed = 0
}

void loop() {
  // Set the motor to spin forward
  digitalWrite(in1Pin, HIGH); // Control pin 1 HIGH
  digitalWrite(in2Pin, LOW);  // Control pin 2 LOW

  // Set the motor speed using PWM (range: 0 - 255)
  int speed = 150;            // PWM value to set motor speed
  analogWrite(enPin, speed);  // Write PWM value to enable pin

  // Keep the motor running for 5 seconds
  delay(5000);                // Pause execution for 5000 ms (5 seconds)

  // Stop the motor
  digitalWrite(in1Pin, LOW);  // Set control pin 1 to LOW
  digitalWrite(in2Pin, LOW);  // Set control pin 2 to LOW
  analogWrite(enPin, 0);      // Set PWM to 0, motor speed = 0

  // Wait for 2 seconds before repeating the loop
  delay(2000);                // Pause execution for 2000 ms (2 seconds)
}
