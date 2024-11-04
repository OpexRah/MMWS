bool isWallPresent(Ultrasonic sensor, float threshold) {
  // Get the distance from the ultrasonic sensor
  float distance = sensor.read(CM);

  // Check if the distance is less than or equal to the threshold
  if (distance > 0 && distance <= threshold) {
    return true;  // Wall is present
  } else {
    return false; // No wall
  }
}