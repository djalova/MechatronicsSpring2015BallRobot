
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;

// operating states
typedef enum {STANDBY, PICKUP, SCORING, DEBUG} mode;

// MODIFY THIS FOR STARTING STATE
mode OP_MODE = PICKUP;

// Keeps track of if robot is rotating to find balls
boolean hasTarget = false;
boolean prevForward = false;

// threshold for center of vision
const int PICKUP_CENTER_THRESHOLD = 30;
const int SCORE_CENTER_THRESHOLD = 60;

// iterations to sample Pixy
const int PIXY_ITERATIONS = 10;

// iterations to sample Ping and threshold
const int PING_ITERATIONS = 5;
const float PING_THRESHOLD = 200.0;
// iterations to run centering algorithm and threshold for center
const int PING_CENTER_ITERATIONS = 15;
const float PING_CENTER_THRESHOLD = 70.0;

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
const int HAND_OFF_SIG = 5;

//NOTE: AVOID USING PINS 50-53 (FOR MEGA). These pins are ICSP and will be used up by the PixyCam.
//NOTE: AVOID USING PINS 10-13 (FOR UNO).
// output pins for drive wheels
const int LEFT_WHEEL_PIN_1 = 8;
const int LEFT_WHEEL_PIN_2 = 9;
const int RIGHT_WHEEL_PIN_1 = 10;
const int RIGHT_WHEEL_PIN_2 = 11;

// output pins for brush
const int BRUSH_PIN_1 = 2;
const int BRUSH_PIN_2 = 3;

// pin for servo
const int SERVO_PIN = 5;

// trigger and echo pins for PING sensor
const int TRIGGER_PIN = 12;
const int ECHO_PIN = 13;

// timeout to go to scoring state
const unsigned long SCORE_TIMEOUT = 45000;
const unsigned long ROTATE_TIMEOUT = 10000;

const int SPEED = 120;

const int PICKUP_ROTATE_ANGLE = 150;
const int PICKUP_DRIVE_ANGLE = 100;
const int SCORE_ANGLE = 180;

unsigned long scoreTimeoutStartTime;
unsigned long rotateTimeoutStartTime;

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

  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
}

void loop() {

  if (OP_MODE == STANDBY) {
    Serial.println("Standby Mode");
    if (checkForFlag() == true) {
      OP_MODE = PICKUP;
      centerRobot();
      scoreTimeoutStartTime = millis();
      rotateTimeoutStartTime = millis();
    }
  }

  else if (OP_MODE == PICKUP) {
    Serial.println("Pickup Mode");
    pickupBalls();
    
    if (!hasTarget) {
      if (millis() - rotateTimeoutStartTime >= ROTATE_TIMEOUT) {
        Serial.println("rotate timeout is over");
        OP_MODE = SCORING;
        rotateTimeoutStartTime = millis();
      }      
      if (millis() - scoreTimeoutStartTime >= SCORE_TIMEOUT) {
        Serial.println("score timeout is over");
        OP_MODE = SCORING;
        scoreTimeoutStartTime = millis();
      }            
    }    
  }
  
  else if (OP_MODE == SCORING) {   
    scoreBalls();
    OP_MODE = PICKUP;
    scoreTimeoutStartTime = millis();
    rotateTimeoutStartTime = millis();
  }

  else if (OP_MODE == DEBUG) {
    centerRobot();
    /*servo.write(180);
    Block *block = getMaxBlock(WALL_SIG, 150);
    if (block != NULL) {
      Serial.println(block->x);
      Serial.println(block->width);
      Serial.println(block->height);
      Serial.println("----");
    } else {
      Serial.println("nothing");
    }
    delay(350);*/

    /*centerRobot();
    Serial.println("finished one iteration");
    turnRobotOff();
    delay(5000);*/
  }
}

boolean checkForFlag() {
  turnRobotOff();
  Block* block = getMaxBlock(HAND_OFF_SIG, PIXY_ITERATIONS);

  if (block != NULL) {
    return true;
  }
}

void pickupBalls() {

  turnRobotOff();
  delay(150);

  Block* block = getMaxBlock(GAME_BALL_SIG, PIXY_ITERATIONS);

  if (block != NULL) {
    hasTarget = true;
    servo.write(PICKUP_DRIVE_ANGLE);
    delay(20);

    Serial.print(block->x);
    Serial.print(" ");
    turnBrushForward();

    if (block->x > (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + PICKUP_CENTER_THRESHOLD)) {
      Serial.println("in middle third");
      turnRobotForward();
      delay(750);
      
      prevForward = true;
    } else if (block->x <= (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD)) {
      Serial.println("turning left");
      float center = MAX_WIDTH / 2.0;
      float turnAmount = (center - (float)block->x) / center;
      Serial.print("turn amount ");
      Serial.println(turnAmount * 200);
      turnRobotLeft();
      delay(turnAmount * 200);
      turnRobotForward();
      delay(100);
      
      prevForward = false;
    } else if (block->x >= (MAX_WIDTH / 2.0 + PICKUP_CENTER_THRESHOLD)) {
      Serial.println("turning right");
      float center = MAX_WIDTH / 2.0;
      float turnAmount = ((float)block->x - center) / center;
      Serial.print("turn amount ");
      Serial.println(turnAmount * 200);
      turnRobotRight();
      delay(turnAmount * 200);
      turnRobotForward();
      delay(100);
      
      prevForward = false;
    }
    
    rotateTimeoutStartTime = millis();    
  } else {
    
    if (prevForward) {
      turnBrushForward();
      turnRobotForward();
      delay(500);
      prevForward = false; 
    }
    
    hasTarget = false;
    servo.write(PICKUP_ROTATE_ANGLE);
    delay(20);

    // Robot should rotate and scan for balls
    rotateRobot();
    turnBrushOff();
    Serial.println("Rotating robot");
    delay(150);
  }
}

void centerRobot() {
  for (int i = 0; i < PING_CENTER_ITERATIONS; i++) {
    float distance = getMaxPingDistance();
    Serial.println(distance);
    if (distance >= PING_CENTER_THRESHOLD) {
      turnRobotForward();
      float forwardAmount = (distance - PING_CENTER_THRESHOLD) / PING_CENTER_THRESHOLD;
      Serial.println(forwardAmount);
      delay(forwardAmount * 2000);
    }
    rotateRobot();
    delay(150);
  }
}

// score balls into goal
void scoreBalls() {

  servo.write(SCORE_ANGLE);
  delay(20);
  delay(1000);
  Serial.println("IN SCORE MODE");

  turnBrushOff();

  centerRobot();

  // rotate until robot finds goal
  Serial.println("score balls");
  unsigned long scoreStartTime = millis();

  while (true) {

    turnRobotOff();
    delay(350);

    Block* block = getMaxBlock(GOAL_SIG, PIXY_ITERATIONS);
    if (block != NULL) {

      Serial.print("x ");
      Serial.print(block->x);
      Serial.print("width ");
      Serial.print(block->width);
      Serial.print("height ");
      Serial.println(block->height);

      if (block->x > (MAX_WIDTH / 2.0 - SCORE_CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + SCORE_CENTER_THRESHOLD)) {

        float distance = getMaxPingDistance();
        if (distance <= 45) {
          break;
        }

        Serial.println("in middle third");
        turnRobotForward();
        delay(300);

      } else if (block->x <= (MAX_WIDTH / 2.0 - SCORE_CENTER_THRESHOLD)) {
        Serial.println("turning left");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = (center - (float)block->x) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        turnRobotLeft();
        delay(turnAmount * 250);
        turnRobotForward();
        delay(100);

      } else if (block->x >= (MAX_WIDTH / 2.0 + SCORE_CENTER_THRESHOLD)) {
        Serial.println("turning right");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = ((float)block->x - center) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        turnRobotRight();
        delay(turnAmount * 250);
        turnRobotForward();
        delay(100);

      }
    }
    else {
      Serial.println("Rotating robot");
      rotateRobot();
      delay(150);

    }
  }

  turnBrushBack();
  turnRobotForward();
  delay(1500);
  turnRobotOff();
  turnBrushBack();
  delay(1000);
  turnRobotBack();
  delay(1500);
}

Block* getMaxBlock(int inputSig, int iterations) {
  boolean found = false;

  unsigned int maxArea = MAX_WIDTH * MAX_HEIGHT;

  for (int i = 0; i < iterations * 50; i++) {
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

float getMaxPingDistance() {
  float maxValue = 0;
  for (int i = 0; i < PING_ITERATIONS; i++) {
    float currentValue = getPingDistance();
    if (currentValue > maxValue && currentValue <= PING_THRESHOLD) {
      maxValue = currentValue;
    }
    delay(20);
  }
  return maxValue;
}

float getPingDistance() {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  float duration, cm;

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);

  // convert the time into a distance
  cm = duration / 29.0 / 2.0;
  return cm;
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
