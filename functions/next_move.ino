void next_move(int* best_x, int* best_y) {
  int min_distance = INF;
  *best_x = -1;
  *best_y = -1;

  // Iterate through all possible movements (up, down, left, right)
  for (int i = 0; i < 4; i++) {
    int new_x = bot_x + movements[i][0];
    int new_y = bot_y + movements[i][1];

    // Check if the new cell is within the bounds and accessible
    if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols && is_accessible(bot_x, bot_y, new_x, new_y)) {
      // Check if the new cell has a smaller distance
      if (distances[new_x][new_y] < min_distance) {
        min_distance = distances[new_x][new_y];
        *best_x = new_x;
        *best_y = new_y;
      }
    }
  }
}