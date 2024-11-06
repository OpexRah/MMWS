// Motor driver pins
const int enPin = 9;   // Enable pin for motor speed (PWM pin)
const int in1Pin = 2;  // Control pin 1
const int in2Pin = 3;  // Control pin 2

void setup() {
  // Set control pins as outputs
  pinMode(enPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Initially stop the motor
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enPin, 0);
}

void loop() {
  // Set motor direction and speed
  digitalWrite(in1Pin, HIGH); // Motor forward
  digitalWrite(in2Pin, LOW);  // Motor forward
  int speed = 150;            // PWM value (0 to 255)
  analogWrite(enPin, speed);  // Set motor speed

  // Run the motor for 5 seconds
  delay(5000);

  // Stop the motor
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enPin, 0);

  // Wait for 2 seconds before running again
  delay(2000);
}
