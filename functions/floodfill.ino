void floodfill() {
  int temp_x, temp_y;
  // Initialize distances to INF
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      distances[i][j] = INF;
    }
  }
  // Set the distance of the goal cell to 0
  distances[goal_x][goal_y] = 0;

  enqueue(goal_x, goal_y);
  while (!isEmpty()) {
    Cell cell = dequeue();
    temp_x = cell.x;
    temp_y = cell.y;

    for (int i = 0; i < 4; i++) {
      int new_x = temp_x + movements[i][0];
      int new_y = temp_y + movements[i][1];

      if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols) {
        if (is_accessible(temp_x, temp_y, new_x, new_y)) {
          int new_distance = distances[temp_x][temp_y] + 1;
          if (distances[new_x][new_y] > new_distance) {
            distances[new_x][new_y] = new_distance;
            enqueue(new_x, new_y);
          }
        }
      }
    }
  }
}