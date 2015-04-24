
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;

// threshold for rotating brush
const int BRUSH_BLOCK_THRESHOLD = 40;

// max width and height of image returned by Pixy
const int MAX_WIDTH = 320;
const int MAX_HEIGHT = 200;

// signatures of game and foul balls
// game ball is green, foul ball is blue
const int GAME_BALL_SIG = 1;
const int FOUL_BALL_SIG = 2;
// signature of goal and wall
const int GOAL_SIG = 3;
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
const int SERVO_PIN = 7;

// pin for ir sensor (if needed)
// const int IR_PIN = 1;

// timeout to go to scoring state
const unsigned int SCORE_TIMEOUT = 30000;
// time to dump balls into goal
const unsigned int SCORE_TIME = 7000;

// time to backup and turn for avoiding wall
const unsigned int WALL_BACKUP_TIME = 2000;
const float WALL_TURN_TIME = 1000;

const int SPEED = 200;

const int STEEP_ANGLE = 120;
const int SHALLOW_ANGLE = 180;

int numBallsCollected = 0;
unsigned long startTime = millis();

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
  //pinMode(LEFT_SWITCH_PIN, INPUT_PULLUP);
  //pinMode(RIGHT_SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  pickupBalls();
  /*servo.write(SHALLOW_ANGLE);
  delay(1000);
  Block *average = getMaxBlo(GOAL_SIG);
  if (average != NULL) {
    Serial.println(average->x); 
  } else {
    Serial.println("nothing");
  }*/
  
}

void pickupBalls() {
  
  servo.write(STEEP_ANGLE);
  turnRobotOff();
  delay(200);
  
  //float observations = numObservations(GAME_BALL_SIG);
  //Serial.println(observations);
  Block* average = getAverageBlock(GAME_BALL_SIG);

  if (average != NULL) {

    Serial.print(average->x);
    Serial.print(" ");
    
    if ( average->x > (MAX_WIDTH / 3.0) && average->x < (2 * MAX_WIDTH / 3.0)) {
      // Go straight if ball is in the middle third
      turnRobotForward();
      turnBrushForward();
      Serial.println("Moving forward");
      delay(1500);
      turnRobotOff();
      turnBrushForward();
      delay(1000);
    }
    else if ( average->x <= (MAX_WIDTH / 3.0)) {

      // Turn right if ball is in the leftmost third of its vision
      turnRobotLeft();
      turnBrushOff();
      Serial.println("Turning left");
      delay(100);
    }
    else if (average->x >= (2 * MAX_WIDTH / 3.0)){
      // Turn left if ball is in the rightmost third
      turnRobotRight();
      turnBrushOff();
      Serial.println("Turning right");
      delay(100);
    }
/*
    // Spin brush if ball is reasonably close
    if (block->width > BRUSH_BLOCK_THRESHOLD && block->height > BRUSH_BLOCK_THRESHOLD) {
      turnBrushForward();
    } else {
      turnBrushOff();
    }
*/
  } else {
    if (millis() - startTime >= SCORE_TIMEOUT) {
      scoreBalls();
      startTime = millis();
    }
    // Robot should rotate and scan for balls
    rotateRobot();
    turnBrushOff();
    Serial.println("Rotating robot");
    delay(150);
  }
  /*
  // Turn towards goal if timed out
  if (millis() - startTime >= SCORE_TIMEOUT && numBallsCollected >= 0) {
    scoreBalls();
    startTime = millis();
    numBallsCollected = 0;
  }
  */

  /*
  // Avoid wall if switch was triggered
  if (digitalRead(LEFT_SWITCH_PIN) == LOW) {
    avoidWallBack();
    avoidWallRight();
  } else if (digitalRead(RIGHT_SWITCH_PIN == LOW)) {
    avoidWallBack();
    avoidWallLeft();
  }
  */

  // Needs a slight delay for some reason. A delay of 10ms makes it
  // rotate for too long. Need to play with these values.

  //turnBrushForward();

  //testMotorFunctions();
}

/*
float getCenterValue(Block **array) {
  Serial.println(array[0]->x);
  Serial.println(array[1]->x);
  return ((array[0])->x + (array[1])->x) / 2.0;
}*/

Block* getAverageBlock(int inputSig) {
  float totalx = 0.0;
  float totalwidth = 0.0;
  float totalheight = 0.0;
  float count = 0;
  Block *avgBlock = new Block();
  for (int i = 0; i < 3; i++) {
    Block* block = getMaxBlock(inputSig);
    if (block != NULL) {
      totalx += block->x;
      totalwidth += block->width;
      totalheight += block->height;
      count++;
    }
  }
  if (count == 0) return NULL;
  avgBlock->x = totalx / count;
  avgBlock->width = totalwidth / count;
  avgBlock->height = totalheight / count;
  return avgBlock;
}

Block* getMaxBlock(int inputSig) {
  int i = 0;
  char buf[32];
  Block* maxBlock = NULL;

  // grab blocks!
  uint16_t blocks = pixy.getBlocks();
  unsigned int maxArea = MAX_WIDTH * MAX_HEIGHT;
  // If there are detect blocks, print them!

  if (blocks)
  {
    sprintf(buf, "Detected %d:\n", blocks);
    //Serial.print(buf);

    // find maximum block size, that is a game ball
    for (int j = 0; j < blocks; j++)
    {
      Block block = pixy.blocks[j];
      //sprintf(buf, "  block %d: ", j);
      //Serial.println(buf);
      if (block.signature == inputSig) {
        //Serial.println(inputSig);
        unsigned int ballArea = block.width * block.height;
        if (ballArea < maxArea) {
          maxArea = ballArea;
          maxBlock = &block;
        }
      }
      //      pixy.blocks[j].print();
    }
  }
  return maxBlock;
}

Block* getAverageAllBlocks(int inputSig) {

  // grab blocks!
  uint16_t blocks = pixy.getBlocks();
  // If there are detect blocks, print them!

  if (blocks)
  {
    Block* avgBlock = new Block();
    float totalx = 0.0;
    float totalwidth = 0.0;
    float totalheight = 0.0;
    float count = 0;

    // find maximum block size, that is a game ball
    for (int j = 0; j < blocks; j++)
    {
      Block block = pixy.blocks[j];
      //sprintf(buf, "  block %d: ", j);
      //Serial.println(buf);
      if (block.signature == inputSig) {
        totalx += block.x;
        totalwidth += block.width;
        totalheight += block.height;
        count++;
      }
    }
    if (count == 0.0) return NULL;
    avgBlock->x = totalx / count;
    avgBlock->width = totalwidth / count;
    avgBlock->height = totalheight / count;
    return avgBlock;
  } else {
    Serial.println("no blocks");
    return NULL; 
  }
}

float getTwoMaxBlocks(int inputSig) {
  
  Block* array[2];
  int i = 0;
  char buf[32];
  int x1 = -1;
  int x2 = -1;

  // grab blocks!
  uint16_t blocks = pixy.getBlocks();
  unsigned int maxArea = MAX_WIDTH * MAX_HEIGHT;
  int index = -1;
  // If there are detect blocks, print them!

  if (blocks)
  {
    sprintf(buf, "Detected %d:\n", blocks);

    // find maximum block size, that is a game ball
    for (int j = 0; j < blocks; j++)
    {
      Block block = pixy.blocks[j];
      if (block.signature == inputSig) {
        //Serial.println(inputSig);
        unsigned int ballArea = block.width * block.height;
        if (ballArea < maxArea) {
          maxArea = ballArea;
          x1 = block.x;
          Serial.print(" x1 ");
          Serial.println(x1);
          index = j;
        }
      }
    }
    maxArea = MAX_WIDTH * MAX_HEIGHT;

    for (int j = 0; j < blocks; j++)
    {
      Block block = pixy.blocks[j];
      if (block.signature == inputSig && j != index) {
        //Serial.println(inputSig);
        unsigned int ballArea = block.width * block.height;
        if (ballArea < maxArea) {
          maxArea = ballArea;
          x2 = block.x;
          Serial.print(" x2 ");
          Serial.println(x2);
          index = j;
        }
      }
    }
    
    if (x1 != -1.0 && x2 == -1.0) {
      return x1;
    } else if (x2 != -1.0 && x1 == -1.0) {
      return x2; 
    }
    
    return ((x1 + x2) / 2.0);    
  }
  return -1.0;
}

// functions to avoid wall
void avoidWallBack() {
  unsigned long avoidStartTime = millis();
  while (millis() - avoidStartTime <= WALL_BACKUP_TIME) {
    turnRobotBack();
  }
}
void avoidWallLeft() {
  unsigned long avoidStartTime = millis();
  while (millis() - avoidStartTime <= WALL_TURN_TIME) {
    turnRobotLeft();
  }
}
void avoidWallRight() {
  unsigned long avoidStartTime = millis();
  while (millis() - avoidStartTime <= WALL_TURN_TIME) {
    turnRobotRight();
  }
}

// score balls into goal
void scoreBalls() {
  servo.write(SHALLOW_ANGLE);
  delay(500);
  turnRobotOff();
  turnBrushOff();
  delay(200);
  // rotate until robot finds goal
  Serial.println("score balls");
  while (true) {
    turnRobotOff();
    delay(150);
    
    Block *average = getAverageBlock(GOAL_SIG);
    
    if (average != NULL) {
      Serial.println(average->x);
    }
    if (average != NULL && average->x > (MAX_WIDTH / 2.0 - 30) && average->x < (MAX_WIDTH / 2.0 + 30)) {
      Serial.println("in middle third");
      break;
    }
    
    Serial.println("Rotating robot");
    rotateRobot();
    delay(50);
  }
  while (true) {
    turnRobotOff();
    delay(150);
    
    Serial.println("moving forward");
    turnRobotForward();
    turnBrushBack();
    delay(1000);
    
    Block *average = getAverageBlock(GOAL_SIG);
    Serial.print("average ");
    Serial.print(average == NULL);
    Serial.print(average->width * average->height);
    
    if (average == NULL) {
      Serial.println("turning brush back");
      turnBrushBack();
      delay(5000);
      turnBrushOff();
      break; 
    }
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
