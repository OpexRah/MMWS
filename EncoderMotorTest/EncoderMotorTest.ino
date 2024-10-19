#include <PinChangeInterrupt.h>

#define encoderPinA 2
#define encoderPinB 3 
#define encoderPinC 4
#define encoderPinD 5 
#define ledPin 13      

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile bool lastA, lastB, lastC, lastD;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinC, INPUT_PULLUP);
  pinMode(encoderPinD, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);  // LED to visually confirm interrupts

  // Use PinChangeInterrupt library to attach interrupts for all encoder pins
  attachPCINT(digitalPinToPCINT(encoderPinA), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinB), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinC), handleEncoderInterruptB, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinD), handleEncoderInterruptB, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // Print encoder count in the main loop
  Serial.print("Encoder A Count: ");
  Serial.print(encoderCountA);
  Serial.print(" Encoder B Count: ");
  Serial.println(encoderCountB);
  
  delay(100);  // Small delay to avoid serial flooding
}

// Interrupt handler for Encoder A (pins A and B)
void handleEncoderInterruptA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

  // Turn on LED when ISR is triggered to check for activity
  digitalWrite(ledPin, HIGH);

  // Handle encoder A logic
  if (A != lastA) {  // A changed
    if (A == B) {
      encoderCountA++;
    } else {
      encoderCountA--;
    }
  }
  lastA = A;
  lastB = B;

  // Turn off LED after ISR is processed
  digitalWrite(ledPin, LOW);
}

// Interrupt handler for Encoder B (pins C and D)
void handleEncoderInterruptB() {
  bool C = digitalRead(encoderPinC);
  bool D = digitalRead(encoderPinD);

  // Turn on LED when ISR is triggered to check for activity
  digitalWrite(ledPin, HIGH);

  // Handle encoder B logic
  if (C != lastC) {  // C changed
    if (C == D) {
      encoderCountB++;
    } else {
      encoderCountB--;
    }
  }
  lastC = C;
  lastD = D;

  // Turn off LED after ISR is processed
  digitalWrite(ledPin, LOW);
}
