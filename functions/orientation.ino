void orient_bot(int target_x, int target_y){
  int old_orientation = orientation;
  if (bot_x == target_x){
    if (bot_y > target_y){
      orientation = 3;
    } else {
      orientation = 1;
    }
  } else if (bot_y == target_y){
    if (bot_x > target_x){
      orientation = 0;
    } else {
      orientation = 2;
    }
  }

  if (old_orientation == 0){
    if (orientation == 1){
      turnRight();
    } else if (orientation == 2){
      one80();
    } else if (orientation == 3){
      turnLeft();
    }
  } else if (old_orientation == 1){
    if (orientation == 0){
      turnLeft();
    } else if (orientation == 2){
      turnRight();
    } else if (orientation == 3){
      one80();
    }
  } else if (old_orientation == 2){
    if (orientation == 0){
      one80();
    } else if (orientation == 1){
      turnLeft();
    } else if (orientation == 3){
      turnRight();
    }
  } else if (old_orientation == 3){
    if (orientation == 0){
      turnRight();
    } else if (orientation == 1){
      one80();
    } else if (orientation == 2){
      turnLeft();
    }
  }
}