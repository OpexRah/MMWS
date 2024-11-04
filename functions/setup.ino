void setup() {
  // setup encoders
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinC, INPUT_PULLUP);
  pinMode(encoderPinD, INPUT_PULLUP);

  // Use PinChangeInterrupt library to attach interrupts for all encoder pins
  attachPCINT(digitalPinToPCINT(encoderPinA), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinB), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinC), handleEncoderInterruptB, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinD), handleEncoderInterruptB, CHANGE);

  // setup motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.begin(115200);
  Serial.println("Starting maze solver in 5 seconds");
  delay(2000);
}