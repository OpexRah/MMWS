void moveForward(int targetDistance) {
  //resetEncoders();
  // kickstart the motor
  leftFwd(150);
  rightFwd(150);
  delay(50);
  while (true) {
    int avgEncoderCount = (encoderCountA + encoderCountB) / 2;
    int currentAngle = encoderCountA - encoderCountB; // Angle difference from encoders

    // Calculate PD outputs for distance and angle
    float distanceOutput = calculateDistancePD(targetDistance, avgEncoderCount);
    float angleOutput = calculateAnglePD(0, encoderCountA, encoderCountB, Kp_angle, Kd_angle);

    // Combine distance and angle outputs to control motors
    int leftMotorSpeed = constrain(baseSpeed + distanceOutput + angleOutput, motorMinSpeed, motorMaxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + distanceOutput - angleOutput, motorMinSpeed, motorMaxSpeed);

    // Drive motors
    leftFwd(leftMotorSpeed);
    rightFwd(rightMotorSpeed);

    // Check if target distance is reached
    if (abs(avgEncoderCount - targetDistance) < 150) break;

    // Debugging information
    Serial.print("Angle: ");
    Serial.print(angleOutput);
    Serial.print("Encoder A ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  leftFwd(0);
  rightFwd(0);
  //resetEncoders();
}