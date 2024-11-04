void loop() {
  Serial.println("Checking walls");
  bool leftwall = isWallPresent(s1, thresholdDistance);
  bool frontwall = isWallPresent(s2, thresholdDistance);
  bool rightwall = isWallPresent(s3, thresholdDistance);

  Serial.print("Left wall: ");
  Serial.print(leftwall);
  Serial.print(" Front wall: ");
  Serial.print(frontwall);
  Serial.print(" Right wall: ");
  Serial.println(rightwall);

  update_walls(leftwall, rightwall, frontwall);

  if (bot_x == goal_x && bot_y == goal_y){
    Serial.println("Goal reached");
    while(1);
  }
  delay(2000);
  Serial.println("Flooding maze");
  floodfill();
  print_distances();
  int best_x, best_y;
  next_move(&best_x, &best_y);
  Serial.print("Next move: ");
  Serial.print(best_x);
  Serial.print(", ");
  Serial.println(best_y);
  Serial.println("Orienting bot");
  orient_bot(best_x, best_y);
  delay(2000);
  Serial.println("Moving bot to next cell");
  delay(1000);
  moveForward(((encoderCountA + encoderCountB) / 2) + cellDistance);
  bot_x = best_x;
  bot_y = best_y;
  Serial.print("Bot position: ");
  Serial.print(bot_x);
  Serial.print(", ");
  Serial.println(bot_y);
  Serial.println("Waiting for 3 seconds");

  delay(2000);
  // turnLeft();
  // delay(2000);
}