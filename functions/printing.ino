void print_distances() {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (distances[i][j] == INF) {
        Serial.print(" 0 ");
      } else {
        Serial.print(" ");
        Serial.print(distances[i][j]);
        Serial.print(" ");
      }
    }
    Serial.println(); // Move to the next line after each row
  }
}

void print_maze() {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (maze[i][j] == INF) {
        Serial.print(" 0 ");
      } else {
        Serial.print(" ");
        Serial.print(maze[i][j]);
        Serial.print(" ");
      }
    }
    Serial.println(); // Move to the next line after each row
  }
}