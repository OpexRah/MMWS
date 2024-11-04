// PD control outputs
float calculateDistancePD(int targetDistance, int currentDistance) {
  static int prevDistanceError = 0;
  int distanceError = targetDistance - currentDistance;
  float distanceDeriv = distanceError - prevDistanceError;
  
  // PD output for distance
  float distanceOutput = (Kp_distance * distanceError) + (Kd_distance * distanceDeriv);
  prevDistanceError = distanceError;

  return distanceOutput;
}

float calculateAnglePD(int targetAngle, int leftEncoder, int rightEncoder, float Kp_angle, float Kd_angle) {
  static int prevAngleError = 0;
  int angleError = targetAngle - (leftEncoder - rightEncoder);
  float angleDeriv = angleError - prevAngleError;

  // PD output for angle
  float angleOutput = (Kp_angle * angleError) + (Kd_angle * angleDeriv);
  prevAngleError = angleError;

  return angleOutput;
}