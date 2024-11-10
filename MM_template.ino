#include <PinChangeInterrupt.h>
#include <Ultrasonic.h>

// pin definitions
#define encoderPinA 2
#define encoderPinB 3
#define encoderPinC 4
#define encoderPinD 5
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

// ========== MAZE VARIABLES ========== //

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

// ========== QUEUE ========== //

// maze variables setup
typedef struct{
    int x,y;
}Cell;

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

// ========== GLOBAL VARIABLES HARDWARE ========== //

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
float kp_angle_turn = 0.2; // Proportional constant for angle turn
float kd_angle_turn = 0.01; // Derivative constant for angle turn

float prevErrorA = 0; // previous error for encoder A
float intErrorA = 0; // integral error for encoder A
float prevErrorB = 0; // previous error for encoder B
float intErrorB = 0; // integral error for encoder B

// Bot movement variables
int leftTurnTarget = 1750; // target for left turn
int rightTurnTarget = 1660; // target for right turn
int one80TurnTarget = 4000; // target for 180 degree turn
int cellDistance = 1850; // distance to move one cell

// ========== FUNCTION DECLARATIONS ========== //
/* These are declarations of the functions. The working of it hasn't been defined yet.
we will understand the entire code function by function and see how everything comes
together*/

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

// ========== ARDUINO SETUP AND MAIN LOOP ========== //
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
  delay(5000);
}

void loop() {}

// ========== FUNCTION DEFINITIONS ========== //
