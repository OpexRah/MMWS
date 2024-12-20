#include <PinChangeInterrupt.h>
#include <Ultrasonic.h>

// pin definitions
#define encoderPinA 2 // yellow
#define encoderPinB 3 // green
#define encoderPinC 4 // green
#define encoderPinD 5 // yellow
#define trigPin1 A4
#define echoPin1 10
#define trigPin2 A5
#define echoPin2 11
#define trigPin3 A0
#define echoPin3 12

// other macros
#define thresholdDistance 8
#define MAX 36
#define INF 32767


// ultrasonic sensor setup
Ultrasonic s1(trigPin1, echoPin1);
Ultrasonic s2(trigPin2, echoPin2);
Ultrasonic s3(trigPin3, echoPin3);

// maze variables setup
typedef struct{
    int x,y;
}Cell;

int maze[6][6] = {
    {8, 2, 2, 2, 2, 7},
    {1, 15, 15, 15, 15, 3},
    {1, 15, 15, 15, 15, 3},
    {1, 15, 15, 15, 15, 3},
    {1, 15, 15, 15, 15, 3},
    {5, 4, 4, 4, 4, 6}
    }; // 2D array to store the maze

int start_x = 0, start_y = 0; // starting point
int goal_x = 2, goal_y = 2; // goal point
int bot_x = 0, bot_y = 0; // current position of the bot
int orientation = 1; // 0: up, 1: right, 2: down, 3: left
int rows = 6, cols = 6; // dimensions of the maze

int cells_with_left_wall[] = {1,5,8,9,11,13,14,16}; // cells with left wall
int cells_with_right_wall[] = {3,6,7,9,11,12,13,16}; // cells with right wall
int cells_with_up_wall[] = {2,7,8,10,12,13,14,16}; // cells with top wall
int cells_with_down_wall[] = {4,5,6,10,11,12,14,16};

int distances[6][6]; // 2D array to store the distances from the starting point
int movements[4][2]={{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // Movements: up, down, left, right

// Queue functions
Cell queue[MAX];
int front=0, rear=0;

void enqueue(int x, int y){ // x and y are the coordinates to enter
    queue[rear].x = x;
    queue[rear].y = y;
    rear = (rear + 1) % (MAX);
}

Cell dequeue(){
    Cell temp = queue[front];
    front = (front + 1) % (MAX);
    return temp;
}

int isEmpty(){
    return front == rear;
}

// variable setup
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile bool lastA, lastB, lastC, lastD;

// Define L293D motor driver pins
// Motor 2
int ENB = 9;  // Enable pin for Motor 1
int IN3 = 8;  // IN1 for Motor 1
int IN4 = 13;  // IN2 for Motor 1

// Motor 1
int ENA = 6; // Enable pin for Motor 2
int IN1 = A2;  // IN3 for Motor 2
int IN2 = 7;  // IN4 for Motor 2

// Motor control limits
int motorMinSpeed = 0;   // Minimum speed
int motorMaxSpeed = 255; // Maximum speed
int baseSpeed = 120;     // Base speed for both motors
float diffFactor = 1;

// PD constants
float Kp_distance = 0.01;  // Proportional constant for distance
float Kd_distance = 0.1;  // Derivative constant for distance
float Kp_angle = 2.5;     // Proportional constant for angle
float Kd_angle = 0.3;     // Derivative constant for angle
float kp_angle_turn = 0.2;
float kd_angle_turn = 0.01;

float prevErrorA = 0;
float intErrorA = 0;
float prevErrorB = 0;
float intErrorB = 0;

// Bot movement variables
int leftTurnTarget = 1750;
int rightTurnTarget = 1660;
int one80TurnTarget = 4000;
int cellDistance = 1850;

// function definitions
// encoder functions
void handleEncoderInterruptA();
void handleEncoderInterruptB();
void resetEncoders();
// motor spin functions
void leftFwd(int speed);
void leftRev(int speed);
void rightFwd(int speed);
void rightRev(int speed);
// PD control outputs
float calculateDistancePD(int targetDistance, int currentDistance);
float calculateAnglePD(int targetAngle, int leftEncoder, int rightEncoder, float Kp_angle, float Kd_angle);
// bot movement functions
void moveForward(int targetDistance);
void moveBackward(int targetDistance);
void turnLeft();
void turnRight();
void one80();
void orient_bot(int target_x, int target_y);
// Floodfill and maze solving functions
bool isWallPresent(Ultrasonic sensor, float threshold);
bool binary_search(int* arr, int size, int target);
void update_walls(bool L, bool R, bool F);
bool is_accessible(int bot_x, int bot_y, int new_x, int new_y);
void floodfill();
void next_move(int* best_x, int* best_y);
void print_distances();
void print_maze();

void setup() {
  // setup encoders
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinC, INPUT_PULLUP);
  pinMode(encoderPinD, INPUT_PULLUP);

  // Use PinChangeInterrupt library to attach interrupts for all encoder pins
  attachPCINT(digitalPinToPCINT(encoderPinA), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinB), handleEncoderInterruptA, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinC), handleEncoderInterruptB, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinD), handleEncoderInterruptB, CHANGE);

  // setup motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.begin(115200);
  Serial.println("Starting maze solver in 5 seconds");
  delay(2000);
}

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

// Interrupt handler for Encoder A (pins A and B)
void handleEncoderInterruptA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

  // Handle encoder A logic
  if (A != lastA) {  // A changed
    if (A == B) {
      encoderCountA++;
    } else {
      encoderCountA--;
    }
  }
  lastA = A;
  lastB = B;
}

// Interrupt handler for Encoder B (pins C and D)
void handleEncoderInterruptB() {
  bool C = digitalRead(encoderPinC);
  bool D = digitalRead(encoderPinD);

  // Handle encoder B logic
  if (C != lastC) {  // C changed
    if (C == D) {
      encoderCountB++;
    } else {
      encoderCountB--;
    }
  }
  lastC = C;
  lastD = D;
}

void resetEncoders() {
  encoderCountA = 0;
  encoderCountB = 0;
  prevErrorA = 0;
  intErrorA = 0;
  prevErrorB = 0;
  intErrorB = 0;
}

// motor spin functions
void leftFwd(int speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void leftRev(int speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void rightFwd(int speed){
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void rightRev(int speed){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

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

void moveForward(int targetDistance) {
  //resetEncoders();
  // kickstart the motor
  leftFwd(150);
  rightFwd(150);
  delay(50);
  while (true) {
    int avgEncoderCount = (encoderCountA + encoderCountB) / 2;
    int currentAngle = encoderCountA - encoderCountB; // Angle difference from encoders

    // Calculate PD outputs for distance and angle
    float distanceOutput = calculateDistancePD(targetDistance, avgEncoderCount);
    float angleOutput = calculateAnglePD(0, encoderCountA, encoderCountB, Kp_angle, Kd_angle);

    // Combine distance and angle outputs to control motors
    int leftMotorSpeed = constrain(baseSpeed + distanceOutput + angleOutput, motorMinSpeed, motorMaxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + distanceOutput - angleOutput, motorMinSpeed, motorMaxSpeed);

    // Drive motors
    leftFwd(leftMotorSpeed);
    rightFwd(rightMotorSpeed);

    // Check if target distance is reached
    if (abs(avgEncoderCount - targetDistance) < 150) break;

    // Debugging information
    Serial.print("Angle: ");
    Serial.print(angleOutput);
    Serial.print("Encoder A ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  leftFwd(0);
  rightFwd(0);
  //resetEncoders();
}

void moveBackward(int targetDistance) {
  // Reset encoders to start counting from zero
  resetEncoders();

  while (true) {
    int avgEncoderCount = (encoderCountA + encoderCountB) / 2;
      // Drive motors in reverse
    leftRev(130);
    rightRev(130);

    // Check if the target distance is reached (abs(avgEncoderCount - targetDistance) < tolerance)
    if (abs(avgEncoderCount - targetDistance) < 150) break;

    // Debugging information
    Serial.print(" Encoder A: ");
    Serial.print(encoderCountA);
    Serial.print(" Encoder B: ");
    Serial.println(encoderCountB);
  }
  // Stop both motors after moving the target distance
  leftRev(0);
  rightRev(0);
  resetEncoders();
}


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