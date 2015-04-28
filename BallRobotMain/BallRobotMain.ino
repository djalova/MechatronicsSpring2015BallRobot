
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;

// threshold for rotating brush
const int BRUSH_BLOCK_THRESHOLD = 40;

// threshold for center of vision (may need to change to 10)
const int CENTER_THRESHOLD = 30;

// iterations to sample Pixy
const int PIXY_ITERATIONS = 5;

// max width and height of image returned by Pixy
const int MAX_WIDTH = 320;
const int MAX_HEIGHT = 200;

// signatures of game and foul balls
// game ball is green, foul ball is blue
const int GAME_BALL_SIG = 1;
const int FOUL_BALL_SIG = 3;
// signature of goal and wall
const int GOAL_SIG = 2;
const int WALL_SIG = 4;

//NOTE: AVOID USING PINS 50-53 (FOR MEGA). These pins are ICSP and will be used up by the PixyCam.
//NOTE: AVOID USING PINS 10-13 (FOR UNO).
// output pins for drive wheels
const int LEFT_WHEEL_PIN_1 = 3;
const int LEFT_WHEEL_PIN_2 = 5;
const int RIGHT_WHEEL_PIN_1 = 6;
const int RIGHT_WHEEL_PIN_2 = 9;

// output pins for brush
const int BRUSH_PIN_1 = 4;
const int BRUSH_PIN_2 = 2;

// pins for left/right swithes to determine if hitting walls
const int SERVO_PIN = 8;

// timeout to go to scoring state
const unsigned long SCORE_TIMEOUT = 45000;
// time to dump balls into goal
const unsigned int SCORE_TIME = 7000;

// time to backup and turn for avoiding wall
const unsigned int WALL_BACKUP_TIME = 2000;
const float WALL_TURN_TIME = 1000;

const int SPEED = 120;

const int STEEP_ANGLE = 120;
const int SHALLOW_ANGLE = 180;

int numBallsCollected = 0;
unsigned long startTime = millis();

static Block *maxBlock = new Block();

Servo servo;

void setup() {
  Serial.begin(9600);
  pixy.init();
  pinMode(LEFT_WHEEL_PIN_1, OUTPUT);
  pinMode(LEFT_WHEEL_PIN_2, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN_1, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN_2, OUTPUT);
  pinMode(BRUSH_PIN_1, OUTPUT);
  pinMode(BRUSH_PIN_2, OUTPUT);
}

void loop() {
  pickupBalls();
  /*Block *block = getMaxBlock(WALL_SIG);
  if (block != NULL) {
    Serial.println(block->x);
    Serial.println(block->width);
    Serial.println(block->height);
    Serial.println("----");
  } else {
    Serial.println("nothing");
  }
  delay(350);*/
}

void pickupBalls() {

  turnRobotOff();
  delay(150);

  Block* block = getMaxBlock(GAME_BALL_SIG);

  if (block != NULL) {


    Serial.print(block->x);
    Serial.print(" ");
    turnBrushForward();

    if (block->x > (MAX_WIDTH / 2.0 - CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + CENTER_THRESHOLD)) {
      Serial.println("in middle third");
      turnRobotForward();
      delay(750);
    } else if (block->x <= (MAX_WIDTH / 2.0 - CENTER_THRESHOLD)) {
      Serial.println("turning left");
      float center = MAX_WIDTH / 2.0;
      float turnAmount = (center - (float)block->x) / center;
      Serial.print("turn amount ");
      Serial.println(turnAmount * 200);
      turnRobotLeft();
      delay(turnAmount * 200);
      turnRobotForward();
      delay(100);
    } else if (block->x >= (MAX_WIDTH / 2.0 + CENTER_THRESHOLD)) {
      Serial.println("turning right");
      float center = MAX_WIDTH / 2.0;
      float turnAmount = ((float)block->x - center) / center;
      Serial.print("turn amount ");
      Serial.println(turnAmount * 200);
      turnRobotRight();
      delay(turnAmount * 200);
      turnRobotForward();
      delay(100);
    }

  } else {
    if (millis() - startTime >= SCORE_TIMEOUT) {
      Serial.println("timeout is over");
      scoreBalls();
      startTime = millis();
    }
    // Robot should rotate and scan for balls
    rotateRobot();
    turnBrushOff();
    Serial.println("Rotating robot");
    delay(150);
  }
}

// score balls into goal
void scoreBalls() {
  delay(1000);
  Serial.println("IN SCORE MODE");

  turnBrushOff();
  // rotate until robot finds goal
  Serial.println("score balls");
  unsigned long scoreStartTime = millis();

  boolean started = false;

  while (true) {

    // exit if still in loop after 60 seconds
    /*if (millis() - scoreStartTime >= SCORE_TIMEOUT) {
      return;
    }*/

    turnRobotOff();
    delay(350);

    Block* block = getMaxBlock(GOAL_SIG);
    if (block != NULL) {

      Serial.print("width ");
      Serial.print(block->width);
      Serial.print("height ");
      Serial.println(block->height);

      if (block->x > (MAX_WIDTH / 2.0 - CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + CENTER_THRESHOLD)) {

        if (block->width > MAX_WIDTH - 150 && block->height > MAX_HEIGHT - 30) {
          break;
        }

        Serial.println("in middle third");
        turnRobotForward();
        delay(500);
      } else if (block->x <= (MAX_WIDTH / 2.0 - CENTER_THRESHOLD)) {
        Serial.println("turning left");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = (center - (float)block->x) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        turnRobotLeft();
        delay(turnAmount * 250);
        turnRobotForward();
        delay(200);
      } else if (block->x >= (MAX_WIDTH / 2.0 + CENTER_THRESHOLD)) {
        Serial.println("turning right");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = ((float)block->x - center) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        turnRobotRight();
        delay(turnAmount * 250);
        turnRobotForward();
        delay(200);
      }
    }
    else {
      Serial.println("Rotating robot");
      rotateRobot();
      delay(150);
    }
  }
  
  // move back a little
  turnRobotBack();
  delay(150);
  // rotate left first
  rotateRobot();
  delay(500);
  // check to see if wall is there
  turnRobotOff();
  delay(350);
  Block* block = getMaxBlock(WALL_SIG);
  if (block != NULL) {
    // if so, rotate right twice as much
    rotateRobotOther();
    delay(1000); 
  }

  delay(50);
  turnRobotOff();
  turnBrushBack();
  delay(1000);
  turnRobotBack();
  delay(1500);

}


Block* getMaxBlock(int inputSig) {
  boolean found = false;

  unsigned int maxArea = MAX_WIDTH * MAX_HEIGHT;

  for (int i = 0; i < PIXY_ITERATIONS * 50; i++) {
    if (i % 50 == 0) {
      uint16_t blocks = pixy.getBlocks();
      if (blocks) {
        // find maximum block size, that is a game ball
        for (int j = 0; j < blocks; j++)
        {
          Block block = pixy.blocks[j];
          if (block.signature == inputSig) {
            unsigned int ballArea = block.width * block.height;
            if (ballArea <= maxArea) {
              maxArea = ballArea;
              maxBlock->x = block.x;
              maxBlock->width = block.width;
              maxBlock->height = block.height;
              found = true;
            }
          }
        }
      }
    }
  }
  if (!found) {
    return NULL;
  } else {
    return maxBlock;
  }
}

void turnRobotLeft() {
  analogWrite(LEFT_WHEEL_PIN_1, LOW);
  analogWrite(LEFT_WHEEL_PIN_2, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_1, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void turnRobotRight() {
  analogWrite(LEFT_WHEEL_PIN_1, SPEED);
  analogWrite(LEFT_WHEEL_PIN_2, LOW);
  analogWrite(RIGHT_WHEEL_PIN_1, LOW);
  analogWrite(RIGHT_WHEEL_PIN_2, SPEED);
}

void turnRobotForward() {
  analogWrite(LEFT_WHEEL_PIN_1, SPEED);
  analogWrite(LEFT_WHEEL_PIN_2, LOW);
  analogWrite(RIGHT_WHEEL_PIN_1, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void turnRobotBack() {
  analogWrite(LEFT_WHEEL_PIN_1, LOW);
  analogWrite(LEFT_WHEEL_PIN_2, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_1, LOW);
  analogWrite(RIGHT_WHEEL_PIN_2, SPEED);
}

void turnRobotOff() {
  analogWrite(LEFT_WHEEL_PIN_1, LOW);
  analogWrite(LEFT_WHEEL_PIN_2, LOW);
  analogWrite(RIGHT_WHEEL_PIN_1, LOW);
  analogWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void rotateRobot() {
  analogWrite(LEFT_WHEEL_PIN_1, LOW);
  analogWrite(LEFT_WHEEL_PIN_2, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_1, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void rotateRobotOther() {
  analogWrite(LEFT_WHEEL_PIN_1, SPEED);
  analogWrite(LEFT_WHEEL_PIN_2, LOW);
  analogWrite(RIGHT_WHEEL_PIN_1, LOW);
  analogWrite(RIGHT_WHEEL_PIN_2, SPEED);
}

void turnBrushForward() {
  digitalWrite(BRUSH_PIN_1, HIGH);
  digitalWrite(BRUSH_PIN_2, LOW);
}

void turnBrushOff() {
  digitalWrite(BRUSH_PIN_1, LOW);
  digitalWrite(BRUSH_PIN_2, LOW);
}

void turnBrushBack() {
  digitalWrite(BRUSH_PIN_1, LOW);
  digitalWrite(BRUSH_PIN_2, HIGH);
}

void testMotorFunctions() {
  turnRobotForward();
  delay(3000);
  turnRobotLeft();
  delay(3000);
  turnRobotRight();
  delay(3000);
  turnRobotBack();
  delay(3000);
}
