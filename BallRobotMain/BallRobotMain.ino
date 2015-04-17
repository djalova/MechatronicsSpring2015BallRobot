
#include <SPI.h>
#include <Pixy.h>

Pixy pixy;

// threshold for width/height of block detected by Pixy
const int BLOCK_THRESHOLD = 15;

// max width and height of image returned by Pixy
const int MAX_WIDTH = 320;
const int MAX_HEIGHT = 200;

// signatures of game and foul balls
// game ball is green, foul ball is blue
const int GAME_BALL_SIG = 2;
const int FOUL_BALL_SIG = 1;


//NOTE: AVOID USING PINS 50-53 (FOR MEGA). These pins are ICSP and will be used up by the PixyCam.
//NOTE: AVOID USING PINS 10-13 (FOR UNO).
// output pins for drive wheels
const int LEFT_WHEEL_PIN_1 = 6;
const int LEFT_WHEEL_PIN_2 = 7;
const int RIGHT_WHEEL_PIN_1 = 8;
const int RIGHT_WHEEL_PIN_2 = 9;

// output pins for brush
const int BRUSH_PIN_1 = 5;
const int BRUSH_PIN_2 = 6;

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

  Block* block = getPixyDistance();

  if (block != NULL) {

    //Serial.print(block->width);
    //Serial.print(block->height);

    if (block->x < MAX_WIDTH / 3) {
      // Turn right if ball is in the leftmost third of its vision
      turnRobotRight();
      Serial.println("Turning right");
    }
    else if (block->x > (2 / 3.0 * MAX_WIDTH) ) {
      // Turn left if ball is in the rightmost third
      turnRobotLeft();
      Serial.println("Turning left");
    } else {
      // Go straight if ball is in the middle third
      turnRobotForward();
      Serial.println("Moving forward");
    }
  }

  else {
    // Robot should rotate and scan for balls
  }
  
  // Needs a slight delay for some reason. A delay of 10ms makes it
  // rotate for too long. Need to play with these values.
  delay(5);
  //turnBrushForward();

  //testMotorFunctions();
}

// gets closest block that is a game ball
Block* getPixyDistance() {
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
    //    Serial.print(buf);

    // find maximum block size, that is a game ball
    for (int j = 0; j < blocks; j++)
    {
      Block block = pixy.blocks[j];
      //sprintf(buf, "  block %d: ", j);
      //Serial.println(buf);
      if (block.signature == GAME_BALL_SIG) {
        //Serial.println(" game ball ");
        int ballArea = block.width * block.height;
        if (ballArea < maxArea) {
          maxArea = ballArea;
          maxBlock = &block;
        }
      } else if (block.signature == FOUL_BALL_SIG) {
        //        Serial.print(" foul ball ");
      }
      //      pixy.blocks[j].print();
    }
  }
  return maxBlock;
}

void turnRobotRight() {
  digitalWrite(LEFT_WHEEL_PIN_1, LOW);
  digitalWrite(LEFT_WHEEL_PIN_2, HIGH);
  digitalWrite(RIGHT_WHEEL_PIN_1, HIGH);
  digitalWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void turnRobotLeft() {
  digitalWrite(LEFT_WHEEL_PIN_1, HIGH);
  digitalWrite(LEFT_WHEEL_PIN_2, LOW);
  digitalWrite(RIGHT_WHEEL_PIN_1, LOW);
  digitalWrite(RIGHT_WHEEL_PIN_2, HIGH);
}

void turnRobotForward() {
  digitalWrite(LEFT_WHEEL_PIN_1, HIGH);
  digitalWrite(LEFT_WHEEL_PIN_2, LOW);
  digitalWrite(RIGHT_WHEEL_PIN_1, HIGH);
  digitalWrite(RIGHT_WHEEL_PIN_2, LOW);
}

void turnRobotBack() {
  digitalWrite(LEFT_WHEEL_PIN_1, LOW);
  digitalWrite(LEFT_WHEEL_PIN_2, HIGH);
  digitalWrite(RIGHT_WHEEL_PIN_1, LOW);
  digitalWrite(RIGHT_WHEEL_PIN_2, HIGH);
}

void turnBrushForward() {
  digitalWrite(BRUSH_PIN_1, HIGH);
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
