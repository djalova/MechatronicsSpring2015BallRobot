/* 530.421 Mechatronics
   Final Project, Ball Robot Logic
   Alex Diehl, Daniel Jalova, Bo Lei, and Upas Narayan
*/

#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

// pixy object
Pixy pixy;

// operating states
typedef enum {STANDBY, PICKUP, SCORING, DEBUG} mode;
// pickup states
typedef enum {NONE, FORWARD, LEFT, RIGHT, BACK, ROTATE} movement;

// current operating mode
mode OP_MODE = DEBUG;
// previous pickup state
movement LAST_MOVE = NONE;

// threshold for center of vision for pickup mode
const int PICKUP_CENTER_THRESHOLD = 30;
// threshold for center of vision for scoring mode
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
const int FOUL_BALL_SIG = 6;
// signature of goal and wall
const int GOAL_SIG = 2;
const int WALL_SIG = 4;
const int HAND_OFF_SIG = 5;

//NOTE: AVOID USING PINS 50-53 (FOR MEGA). These pins are ICSP and will be used up by the Pixy.
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
// timeout in rotate state (if we have not found balls after rotating for 10 secs)
const unsigned long ROTATE_TIMEOUT = 10000;
// timeout to check if stuck on wall
const unsigned long WALL_CHECK_TIMEOUT = 2000;
// timeout to start robot if relay failed
const unsigned long STANDBY_TIMEOUT = 20000;

// speed to turn drive wheels
const int SPEED = 150;

// servo angle for pickup rotate
const int PICKUP_ROTATE_ANGLE = 150;
// servo angle for pickup left/right/center
const int PICKUP_DRIVE_ANGLE = 100;
// servo angle for scoring
const int SCORE_ANGLE = 180;

// start times for scoring and rotating timeouts
unsigned long scoreTimeoutStartTime;
unsigned long rotateTimeoutStartTime;
// start time for timeout if relay failed
unsigned long standbyTimeoutStartTime;
// used to check if stuck on side of wall
unsigned long wallCheckTimeoutStartTime;
float distanceToWall;
float prevDistanceToWall = 0;
const float  WALL_CHECK_THRESHOLD = 10.0;
const float WALL_CHECK_RANGE = 3.0;


Servo servo;

void setup() {
  Serial.begin(9600);
  pixy.init();
  
  // output for all pins and attach servo
  pinMode(LEFT_WHEEL_PIN_1, OUTPUT);
  pinMode(LEFT_WHEEL_PIN_2, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN_1, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN_2, OUTPUT);
  pinMode(BRUSH_PIN_1, OUTPUT);
  pinMode(BRUSH_PIN_2, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  standbyTimeoutStartTime = millis();
}

void loop() {

  if (OP_MODE == STANDBY) {
    Serial.println("Standby Mode");
    // if in standby, check for flag or if 20-sec timeout is over
    // then change to pickup state, move backwards, and start timeouts
    if (checkForFlag() == true || millis() - standbyTimeoutStartTime >= STANDBY_TIMEOUT) {
      OP_MODE = PICKUP;
      turnRobotBack();
      delay(1500);
      scoreTimeoutStartTime = millis();
      rotateTimeoutStartTime = millis();
      wallCheckTimeoutStartTime = millis();
    }
  }

  else if (OP_MODE == PICKUP) {
    Serial.println("Pickup Mode");    

    // timeout if we are not rotating
    // transition to scoring state if we didn't find balls after rotating for 10 secs
    // or 30 seconds is up
    if (LAST_MOVE != ROTATE) {
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

    /*
    Below is wall recovery code that did not work.
    
    if (millis() - wallCheckTimeoutStartTime >= WALL_CHECK_TIMEOUT) {
      // find distance to wall
      distanceToWall = getMaxPingDistance();
      
      Serial.print("Curr distance to wall: ");
      Serial.println(distanceToWall);

      Serial.print("Prev distance to wall: ");
      Serial.println(prevDistanceToWall);
     
      // check if distance has changed by more than 3 cm or less than 10 cm
      if ( distanceToWall <= prevDistanceToWall + WALL_CHECK_RANGE && distanceToWall >= prevDistanceToWall - WALL_CHECK_RANGE
           && distanceToWall <  WALL_CHECK_THRESHOLD) {
        Serial.println("Stuck at wall. Attempting to recover.");
        // attempt to recover from wall
        wallRecovery();
      }
      // save previous distance and restart timeout
      prevDistanceToWall = distanceToWall;
      wallCheckTimeoutStartTime = millis();
    }*/
 
    pickupBalls();

  }

  else if (OP_MODE == SCORING) {
    
    // score balls, change to pickup, restart timeouts
    scoreBalls();
    OP_MODE = PICKUP;
    scoreTimeoutStartTime = millis();
    rotateTimeoutStartTime = millis();
  }

  else if (OP_MODE == DEBUG) {
    // debugging code, checking readings from Pixy and Ping
    Block *block = getMaxBlock(GAME_BALL_SIG, PIXY_ITERATIONS);
    if (block != NULL) {
      Serial.println("--GAME BALL--");
      Serial.println(block->x);
      Serial.println(block->width);
      Serial.println(block->height);
      Serial.println("----");
    } else {
      Serial.println("nothing");
    }
    delay(350);

    block = getMaxBlock(FOUL_BALL_SIG, PIXY_ITERATIONS);
    if (block != NULL) {
      Serial.println("--FOUL BALL--");
      Serial.println(block->x);
      Serial.println(block->width);
      Serial.println(block->height);
      Serial.println("----");
    } else {
      Serial.println("nothing");
    }
    delay(350);

    /*centerRobot();
    Serial.println("finished one iteration");
    turnRobotOff();
    delay(5000);*/
  }
}


/* Checks for flag to perform relay between pipe and ball bots.
*/
boolean checkForFlag() {
  
  // turn robot off, read goal signature (orange is basically red)
  // and return true if signature detected
  turnRobotOff();
  delay(350);
  Block* block = getMaxBlock(GOAL_SIG, PIXY_ITERATIONS);

  if (block != NULL) {
    return true;
  }
}


/* Pick up balls from arena.
*/
void pickupBalls() {

  // turn robot off for a little bit
  turnRobotOff();
  delay(150);

  // get Pixy readings for game and foul balls
  Block* block = getMaxBlock(GAME_BALL_SIG, PIXY_ITERATIONS);
  delay(150);

  Block* bad_block = getMaxBlock(FOUL_BALL_SIG, PIXY_ITERATIONS);
  delay(150);

  // block was detected, exit out of rotate state
  if (block != NULL) {
    
    // write a lower servo angle to pursue ball
    servo.write(PICKUP_DRIVE_ANGLE);
    delay(20);

    Serial.println(block->x);
    Serial.println(bad_block->x);
  
    // spin brush forward
    turnBrushForward();

    // game ball is in center of Pixy's view
    if (block->x > (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + PICKUP_CENTER_THRESHOLD)) {
      Serial.print("Bad block width:");
      Serial.println(bad_block->width);

      Serial.print("Bad block height:");
      Serial.println(bad_block->height);

      Serial.print("Block size:");
      Serial.println(block->width * block->height);

      
      if (bad_block == NULL || bad_block->x < (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD) ||
          bad_block->x > (MAX_WIDTH / 2.0 + PICKUP_CENTER_THRESHOLD) || (block->width * block->height > bad_block->width * bad_block->height) ) {
        // Foul ball is smaller than game ball, can actively pursue game ball
        Serial.println("in middle third");
        turnRobotForward();
        delay(750);
      }
      else if ( block->x < bad_block->x && bad_block->x > (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD)) {
        // Foul ball is left of game ball, turn until bad block is outside of threshold
        Serial.println("Avoid left foul ball");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = (center - (float)block->x) / center;
        turnRobotRight();
        delay(turnAmount * 200);
        turnRobotForward();
        delay(100);
      }
      else if ( block->x > bad_block->x && bad_block->x < (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD)) {
        // Foul ball is right of game ball, turn until bad block is outside of threshold
        Serial.println("Avoid right ball");
        float center = MAX_WIDTH / 2.0;
        float turnAmount = (center - (float)block->x) / center;
        turnRobotLeft();
        delay(turnAmount * 200);
        turnRobotForward();
        delay(100);
      }
      
      LAST_MOVE = FORWARD;

    } else if (block->x <= (MAX_WIDTH / 2.0 - PICKUP_CENTER_THRESHOLD)) {
      Serial.println("turning left");
      float center = MAX_WIDTH / 2.0;
      // turn amount is center minus x reading from Pixy
      float turnAmount = (center - (float)block->x) / center;
      Serial.print("turn amount ");
      // turn left an amount proportional to difference
      Serial.println(turnAmount * 200);
      turnRobotLeft();
      delay(turnAmount * 200);
      // move forward a little bit
      turnRobotForward();
      delay(100);
      
      LAST_MOVE = LEFT;

    } else if (block->x >= (MAX_WIDTH / 2.0 + PICKUP_CENTER_THRESHOLD)) {
      Serial.println("turning right");
      float center = MAX_WIDTH / 2.0;
      // turn amount is center minus x reading from Pixy
      float turnAmount = ((float)block->x - center) / center;
      Serial.print("turn amount ");
      // turn right an amount proportional to difference
      Serial.println(turnAmount * 200);
      turnRobotRight();
      delay(turnAmount * 200);
      // move forward a little bit
      turnRobotForward();
      delay(100);

      LAST_MOVE = RIGHT;
    }

    rotateTimeoutStartTime = millis();
  } else {
    
    // in rotate state (no block detected)
    
    // if last move was going forward, turn brush and move forward for a little bit more
    // to increase odds of picking up ball
    if (LAST_MOVE == FORWARD) {
      turnBrushForward();
      turnRobotForward();
      delay(500);
    }

    // raise servo angle a little bit
    servo.write(PICKUP_ROTATE_ANGLE);
    delay(20);

    // rotate and scan for balls
    rotateRobot();
    turnBrushOff();
    Serial.println("Rotating robot");
    delay(150);
    turnRobotOff();
    
    LAST_MOVE = ROTATE;
  }
}


/* Center robot using Ping sensor.
*/
void centerRobot() {
  // iterate 15 times, rotating by 150 ms each time
  for (int i = 0; i < PING_CENTER_ITERATIONS; i++) {
    // get max distance sensed by Ping
    float distance = getMaxPingDistance();
    Serial.println(distance);
    // if distance sensed is above 65 cm
    if (distance >= PING_CENTER_THRESHOLD) {
      // move forward by an amount proportional to the difference
      // between the Ping reading and the 65 cm
      turnRobotForward();
      float forwardAmount = (distance - PING_CENTER_THRESHOLD) / PING_CENTER_THRESHOLD;
      Serial.println(forwardAmount);
      delay(forwardAmount * 2000);
    }
    rotateRobot();
    delay(150);
  }
}


/* Score balls into the goal.
*/
void scoreBalls() {

  // turn servo completely up (180 degrees)
  servo.write(SCORE_ANGLE);
  delay(20);
  delay(1000);
  Serial.println("IN SCORE MODE");

  // turn the brush off
  turnBrushOff();
  // center robot
  centerRobot();

  Serial.println("score balls");
  unsigned long scoreStartTime = millis();

  while (true) {
    
    // exit if we can't score in 45 secs
    if (millis() - scoreStartTime >= SCORE_TIMEOUT) {
      return; 
    }

    // get goal signature reading from Pixy
    turnRobotOff();
    delay(350);
    Block* block = getMaxBlock(GOAL_SIG, PIXY_ITERATIONS);
    
    // only detect goal if it's actually the goal
    // (small width/height could be a speck of the pipe bot)
    if (block != NULL && block->width > 10 && block->height > 10) {

      // detected goal
      Serial.print("x ");
      Serial.print(block->x);
      Serial.print("width ");
      Serial.print(block->width);
      Serial.print("height ");
      Serial.println(block->height);

      if (block->x > (MAX_WIDTH / 2.0 - SCORE_CENTER_THRESHOLD) && block->x < (MAX_WIDTH / 2.0 + SCORE_CENTER_THRESHOLD)) {
        
        // in center field of view
        
        // read distance from Ping
        float distance = getMaxPingDistance();
        // stop moving towards goal if 45 cm away from goal
        if (distance <= 45) {
          break;
        }

        Serial.println("in middle third");
        // move forward if in center
        turnRobotForward();
        delay(300);

      } else if (block->x <= (MAX_WIDTH / 2.0 - SCORE_CENTER_THRESHOLD)) {
        Serial.println("turning left");
        float center = MAX_WIDTH / 2.0;
        // determine turn amount by difference between center and x value
        float turnAmount = (center - (float)block->x) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        // turn left amount proportional to difference
        turnRobotLeft();
        delay(turnAmount * 250);
        // move forward a little bit
        turnRobotForward();
        delay(100);

      } else if (block->x >= (MAX_WIDTH / 2.0 + SCORE_CENTER_THRESHOLD)) {
        Serial.println("turning right");
        float center = MAX_WIDTH / 2.0;
        // determine turn amount by difference between center and x value
        float turnAmount = ((float)block->x - center) / center;
        Serial.print("turn amount ");
        Serial.println(turnAmount * 250);
        // turn left amount proportional to difference
        turnRobotRight();
        delay(turnAmount * 250);
        // move forward a little bit
        turnRobotForward();
        delay(100);
      }
    }
    else {
      // if didn't find goal, keep rotating until goal is found
      Serial.println("Rotating robot");
      rotateRobot();
      delay(150);

    }
  }
  
  // goal is found and robot has moved towards goal
  
  // turn brush back
  turnBrushBack();
  // go forward for a little bit
  turnRobotForward();
  delay(1500);
  turnRobotOff();
  // spin brush back and go back a little bit
  turnBrushBack();
  delay(1000);
  turnRobotBack();
  delay(1500);
  
  // balls should be dumped in goal now
}


/* Gets maximum block from Pixy given an input signature and iterations for sampling.
*/
Block* getMaxBlock(int inputSig, int iterations) {
  boolean found = false;
  Block* maxBlock = new Block();
  // max area of arena
  unsigned int maxArea = MAX_WIDTH * MAX_HEIGHT;

  for (int i = 0; i < iterations * 50; i++) {
    if (i % 50 == 0) {
      // get blocks from pixy
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
    // return null if no block found
    return NULL;
  } else {
    // otherwise return max block
    return maxBlock;
  }
}


/* Attempt to recover from wall (if in pickup mode).
*/
void wallRecovery() {

  // perform operations to try and avoid the wall
  if (LAST_MOVE == FORWARD) {
    // if last move is forward, try to go back to move forward easier
    turnRobotBack();
    delay(300);
  }
  else if (LAST_MOVE == LEFT) {
    // if last move is left, go right, back, then left
    turnRobotRight();
    delay(100);
    turnRobotBack();
    delay(300);
    turnRobotLeft();
    delay(100);
  }
  else if (LAST_MOVE == RIGHT) {
    // if last move is right, go left, back, then right
    turnRobotLeft();
    delay(100);
    turnRobotBack();
    delay(300);
    turnRobotRight();
    delay(100);
  }
}


/* Get the maximum distance sensed by Ping.
*/
float getMaxPingDistance() {
  float maxValue = 0;
  // sample 10 times
  for (int i = 0; i < PING_ITERATIONS; i++) {
    // get distance from ping
    float currentValue = getPingDistance();
    // check if distance is greater than current max AND
    // distance is less than 200 cm (if distance > 200 cm
    // then reading is wrong since arena's max diagonal distance is ~180cm)
    if (currentValue > maxValue && currentValue <= PING_THRESHOLD) {
      maxValue = currentValue;
    }
    delay(20);
  }
  return maxValue;
}


/* Get distance from 4-pin Ping sensor.
*/
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


/* Turn robot left, right, forward, back, and off.
*/
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

/* Rotate robot (left).
*/
void rotateRobot() {
  analogWrite(LEFT_WHEEL_PIN_1, LOW);
  analogWrite(LEFT_WHEEL_PIN_2, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_1, SPEED);
  analogWrite(RIGHT_WHEEL_PIN_2, LOW);
}

/* Rotate robot in other direction (right).
*/
void rotateRobotOther() {
  analogWrite(LEFT_WHEEL_PIN_1, SPEED);
  analogWrite(LEFT_WHEEL_PIN_2, LOW);
  analogWrite(RIGHT_WHEEL_PIN_1, LOW);
  analogWrite(RIGHT_WHEEL_PIN_2, SPEED);
}

/* Turn brush forward, off, and back.
*/
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

/* Simple test to check motor functions.
*/
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
