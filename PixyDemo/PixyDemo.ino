
#include <SPI.h>  
#include <Pixy.h>

// This is the main Pixy object 
Pixy pixy;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        Block block = pixy.blocks[j];
        if (block.width < 15 || block.height < 15) {
          continue; 
        }
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        if (block.signature == 1) {
          Serial.print(" blue ");
        } else if (block.signature == 2) {
          Serial.print(" green ");
        }
        pixy.blocks[j].print();
      }
    }
  }  
}

