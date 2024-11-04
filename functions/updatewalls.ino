// Floodfill and maze solving functions
bool binary_search(int* arr, int size, int target) {
  int left = 0;
  int right = size - 1;

  while (left <= right) {
    int mid = left + (right - left) / 2;

    if (arr[mid] == target) {
      return true;
    } else if (arr[mid] < target) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }

  return false;
}

void update_walls(bool L, bool R, bool F) {
  /*
  update the walls in the maze array
  :param L: left wall sensor (True if wall is present)
  :param R: right wall sensor (True if wall is present)
  :param F: front wall sensor (True if wall is present)
  */
  if (L == true && R == true && F == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 13;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 12;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 11;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 14;
    }
  } else if (L == true && R == true && F == false) {
    if (orientation == 0 || orientation == 2) {
      maze[bot_x][bot_y] = 9;
    } else if (orientation == 1 || orientation == 3) {
      maze[bot_x][bot_y] = 10;
    }
  } else if (L == true && R == false && F == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 8;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 7;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 6;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 5;
    }
  } else if (L == false && R == true && F == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 7;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 6;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 5;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 8;
    }
  } else if (L == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 1;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 2;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 3;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 4;
    }
  } else if (R == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 3;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 4;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 1;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 2;
    }
  } else if (F == true) {
    if (orientation == 0) {
      maze[bot_x][bot_y] = 2;
    } else if (orientation == 1) {
      maze[bot_x][bot_y] = 3;
    } else if (orientation == 2) {
      maze[bot_x][bot_y] = 4;
    } else if (orientation == 3) {
      maze[bot_x][bot_y] = 1;
    }
  } else {
    maze[bot_x][bot_y] = 15;
  }

  int current_cellwall_type = maze[bot_x][bot_y];

  for (int i = 0; i <= 3; i++) {
    if (i == 0) {
      // up cell
      int cell_x = bot_x - 1, cell_y = bot_y;
      if (cell_x >= 0 && cell_x < rows && cell_y >= 0 && cell_y < cols) {
        if (binary_search(cells_with_up_wall, 8, current_cellwall_type)) {
          int target_cellwall_type = maze[cell_x][cell_y];
          if (binary_search(cells_with_down_wall, 8, target_cellwall_type) == false) {
            bool target_left_present = binary_search(cells_with_left_wall, 8, target_cellwall_type);
            bool target_right_present = binary_search(cells_with_right_wall, 8, target_cellwall_type);
            bool target_front_present = binary_search(cells_with_up_wall, 8, target_cellwall_type);

            for (int j = 0; j < 8; j++) {
              if (binary_search(cells_with_left_wall, 8, cells_with_down_wall[j]) == target_left_present && binary_search(cells_with_right_wall, 8, cells_with_down_wall[j]) == target_right_present &&
                  binary_search(cells_with_up_wall, 8, cells_with_down_wall[j]) == target_front_present) {
                maze[cell_x][cell_y] = cells_with_down_wall[j];
                break;
              }
            }
          }
        }
      }

    } else if (i == 1) {
      // down cell
      int cell_x = bot_x + 1, cell_y = bot_y;
      if (cell_x >= 0 && cell_x < rows && cell_y >= 0 && cell_y < cols) {
        if (binary_search(cells_with_down_wall, 8, current_cellwall_type)) {
          int target_cellwall_type = maze[cell_x][cell_y];
          if (binary_search(cells_with_up_wall, 8, target_cellwall_type) == false) {
            bool target_left_present = binary_search(cells_with_left_wall, 8, target_cellwall_type);
            bool target_right_present = binary_search(cells_with_right_wall, 8, target_cellwall_type);
            bool target_down_present = binary_search(cells_with_down_wall, 8, target_cellwall_type);

            for (int j = 0; j < 8; j++) {
              if (binary_search(cells_with_left_wall, 8, cells_with_up_wall[j]) == target_left_present && binary_search(cells_with_right_wall, 8, cells_with_up_wall[j]) == target_right_present &&
                  binary_search(cells_with_down_wall, 8, cells_with_up_wall[j]) == target_down_present) {
                maze[cell_x][cell_y] = cells_with_up_wall[j];
                break;
              }
            }
          }
        }
      }
    } else if (i == 2) {
      // left cell
      int cell_x = bot_x, cell_y = bot_y - 1;
      if (cell_x >= 0 && cell_x < rows && cell_y >= 0 && cell_y < cols) {
        if (binary_search(cells_with_left_wall, 8, current_cellwall_type)) {
          int target_cellwall_type = maze[cell_x][cell_y];
          if (binary_search(cells_with_right_wall, 8, target_cellwall_type) == false) {
            bool target_left_present = binary_search(cells_with_left_wall, 8, target_cellwall_type);
            bool target_down_present = binary_search(cells_with_down_wall, 8, target_cellwall_type);
            bool target_up_present = binary_search(cells_with_up_wall, 8, target_cellwall_type);

            for (int j = 0; j < 8; j++) {
              if (binary_search(cells_with_down_wall, 8, cells_with_right_wall[j]) == target_down_present && binary_search(cells_with_left_wall, 8, cells_with_right_wall[j]) == target_left_present &&
                  binary_search(cells_with_up_wall, 8, cells_with_right_wall[j]) == target_up_present) {
                maze[cell_x][cell_y] = cells_with_right_wall[j];
                break;
              }
            }
          }
        }
      }
    } else if (i == 3) {
      // right cell
      int cell_x = bot_x, cell_y = bot_y + 1;
      if (cell_x >= 0 && cell_x < rows && cell_y >= 0 && cell_y < cols) {
        if (binary_search(cells_with_right_wall, 8, current_cellwall_type)) {
          int target_cellwall_type = maze[cell_x][cell_y];
          if (binary_search(cells_with_left_wall, 8, target_cellwall_type) == false) {
            bool target_right_present = binary_search(cells_with_right_wall, 8, target_cellwall_type);
            bool target_down_present = binary_search(cells_with_down_wall, 8, target_cellwall_type);
            bool target_up_present = binary_search(cells_with_up_wall, 8, target_cellwall_type);

            for (int j = 0; j < 8; j++) {
              if (binary_search(cells_with_down_wall, 8, cells_with_left_wall[j]) == target_down_present && binary_search(cells_with_right_wall, 8, cells_with_left_wall[j]) == target_right_present &&
                  binary_search(cells_with_up_wall, 8, cells_with_left_wall[j]) == target_up_present) {
                maze[cell_x][cell_y] = cells_with_left_wall[j];
                break;
              }
            }
          }
        }
      }
    }
  }
}