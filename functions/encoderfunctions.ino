// Interrupt handler for Encoder A (pins A and B)
void handleEncoderInterruptA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

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
}

// Interrupt handler for Encoder B (pins C and D)
void handleEncoderInterruptB() {
  bool C = digitalRead(encoderPinC);
  bool D = digitalRead(encoderPinD);

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
}

void resetEncoders() {
  encoderCountA = 0;
  encoderCountB = 0;
  prevErrorA = 0;
  intErrorA = 0;
  prevErrorB = 0;
  intErrorB = 0;
}