void turnLeft() {
  resetEncoders();
  moveForward(((encoderCountA + encoderCountB) / 2) + 500);
  int targetAngle = -leftTurnTarget;  // Target angle for a 90-degree left turn
  
  while (true) {
    int currentAngle = encoderCountA - encoderCountB;
    float angleOutput = calculateAnglePD(targetAngle, encoderCountA, encoderCountB, kp_angle_turn, kd_angle_turn);
    
    // Adjust motor speeds: left motor slightly slower to compensate for offset
    int leftMotorSpeed = constrain(fabs(angleOutput) * diffFactor, motorMinSpeed, 130); 
    int rightMotorSpeed = constrain(fabs(angleOutput), motorMinSpeed, 130);

    leftRev(leftMotorSpeed);    // Left motor reverse
    rightFwd(rightMotorSpeed);  // Right motor forward
    
    if (abs(currentAngle - targetAngle) < 150 || (currentAngle < (targetAngle + 150))) break;

    Serial.print("Encoder A: ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.print(encoderCountB);
    Serial.print(" AngleOutput: ");
    Serial.println(leftMotorSpeed);
  }
  leftFwd(0);
  rightFwd(0);
  resetEncoders();
  moveBackward(((encoderCountA + encoderCountB) / 2) - 450);
}

void turnRight() {
  resetEncoders();
  moveForward(((encoderCountA + encoderCountB) / 2) + 500);
  int targetAngle = rightTurnTarget;  // Target angle for a 90-degree right turn
  
  while (true) {
    int currentAngle = encoderCountA - encoderCountB;  // Difference between encoders

    // Calculate PD output for angle control
    float angleOutput = calculateAnglePD(targetAngle, encoderCountA, encoderCountB, kp_angle_turn, kd_angle_turn);
    
    // Adjust motor speeds: right motor slightly slower to compensate for offset
    int leftMotorSpeed = constrain(fabs(angleOutput), motorMinSpeed, 130);
    int rightMotorSpeed = constrain(fabs(angleOutput) * diffFactor, motorMinSpeed, 130); 

    leftFwd(leftMotorSpeed);    // Left motor forward
    rightRev(rightMotorSpeed);  // Right motor reverse

    // Stop turning if target angle is reached within tolerance
    if (abs(currentAngle - targetAngle) < 150) break;

    // Debugging output
    Serial.print("Encoder A: ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  leftFwd(0);
  rightFwd(0);
  resetEncoders();
  moveBackward(((encoderCountA + encoderCountB) / 2) - 450);
}

void one80() {
  //resetEncoders();
  int targetAngle = one80TurnTarget;  // Target angle for a 180-degree turn
  
  while (true) {
    int currentAngle = encoderCountA - encoderCountB;  // Difference between encoders

    // Calculate PD output for angle control
    float angleOutput = calculateAnglePD(targetAngle, encoderCountA, encoderCountB, kp_angle_turn, kd_angle_turn);

    // Set both motors to opposite directions to turn in place
    int motorSpeed = constrain(fabs(angleOutput), motorMinSpeed, motorMaxSpeed);
    leftFwd(motorSpeed);    // Left motor forward
    rightRev(motorSpeed);   // Right motor in reverse

    // Stop turning if target angle is reached within tolerance
    if (abs(currentAngle - targetAngle) < 150) break;

    // Debugging output
    Serial.print("Encoder A: ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  leftFwd(0);
  rightFwd(0);
  resetEncoders();
}