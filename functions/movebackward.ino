void moveBackward(int targetDistance) {
  // Reset encoders to start counting from zero
  resetEncoders();

  while (true) {
    int avgEncoderCount = (encoderCountA + encoderCountB) / 2;
      // Drive motors in reverse
    leftRev(130);
    rightRev(130);

    // Check if the target distance is reached (abs(avgEncoderCount - targetDistance) < tolerance)
    if (abs(avgEncoderCount - targetDistance) < 150) break;

    // Debugging information
    Serial.print(" Encoder A: ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  // Stop both motors after moving the target distance
  leftRev(0);
  rightRev(0);
  resetEncoders();
}