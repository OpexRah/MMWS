bool is_accessible(int bot_x, int bot_y, int new_x, int new_y) {
  /* Returns True if the mouse can move to new_x, new_y from bot_x, bot_y (two adjacent cells)
     :param: bot_x, bot_y: current cell
     :param: new_x, new_y: adjacent cell
  */
  if (bot_x == new_x) {   // same row
    if (new_y < bot_y) {  // moving left
      if (binary_search(cells_with_left_wall, 8, maze[bot_x][bot_y])) {
        return false;
      }
    } else {  // moving right
      if (binary_search(cells_with_right_wall, 8, maze[bot_x][bot_y])) {
        return false;
      }
    }
  } else if (bot_y == new_y) {  // same column
    if (new_x < bot_x) {        // moving up
      if (binary_search(cells_with_up_wall, 8, maze[bot_x][bot_y])) {
        return false;
      }
    } else {  // moving down
      if (binary_search(cells_with_down_wall, 8, maze[bot_x][bot_y])) {
        return false;
      }
    }
  }
  return true;
}