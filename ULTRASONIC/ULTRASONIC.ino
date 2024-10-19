#include <Ultrasonic.h>

//pin numbers for three ultrasonic sensors
#define trigPin1 A4
#define echoPin1 10

// ultrasonic sensor setup
Ultrasonic s1(trigPin1, echoPin1);

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  Serial.begin(9600); //start the serial communication
}

void loop() {
  //measure distance for each sensor
  float distance = s1.read(CM);

  //print the distances on the Serial Monitor
  Serial.print("Distance : ");
  Serial.println(distance);
  delay(500); //wait for half a second before the next reading
}
