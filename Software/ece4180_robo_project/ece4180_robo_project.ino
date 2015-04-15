//**************************************************************************************************
// Name:
//  ece4180_robo_project.ino
//
// Description:
//  Controls the functionality of a CM-530 controller via Arduino
//
// Changes:
//  Sean Iftody, 2015-Apr-14:
//   Created.
//**************************************************************************************************
//**************************************************************************************************
// INCLUDES
//**************************************************************************************************
#include <SPI.h>  
#include <Pixy.h>

//**************************************************************************************************
// DEFINES
//**************************************************************************************************
#define X_AXIS_MID 160 // Middle point in the x-axis
#define Y_AXIS_MID 100 // Middle point in the y-axis

//**************************************************************************************************
// VARIABLES
//**************************************************************************************************
Pixy pixy; // Create a Pixy camera object

double maxDeviation = 0.75 * (X_AXIS_MID/2); // Determines the max deviation from the center

//**************************************************************************************************
// CONSTANTS
//**************************************************************************************************
// Instructions to move forward, left, right, and back.
const uint8_t  moveFoward[6]  = {0xAA, 0x01, 0x0E, 0xAA, 0x00, 0x1E};
const uint8_t* moveFoward_ptr = moveFoward;
const uint8_t  moveLeft[6]    = {0xAA, 0x04, 0x5E, 0xAA, 0x00, 0x1E};
const uint8_t* moveLeft_ptr   = moveLeft;
const uint8_t  moveRight[6]   = {0xAA, 0x08, 0x1E, 0xAA, 0x00, 0x1E};
const uint8_t* moveRight_ptr  = moveRight;
const uint8_t  moveBack[6]    = {0xAA, 0x02, 0x7E, 0xAA, 0x00, 0x1E};
const uint8_t* moveBack_ptr   = moveBack;

//**************************************************************************************************
// FUNCTION PROTOTYPES
//**************************************************************************************************
void moveRobotForward();
void moveRobotLeft();
void moveRobotRight();
void moveRobotBack();

//**************************************************************************************************
// FUNCTIONS
//**************************************************************************************************
//**************************************************************************************************
// Name: setup
//**************************************************************************************************
void setup() {
  // Initialize serial port (1900 baud)
  Serial.begin(1900);

  // Initialize Pixy camera
  pixy.init();
}

//**************************************************************************************************
// Name: loop
//**************************************************************************************************
void loop()
{
  uint16_t blocks; // Number of blocks found by Pixy camera
  uint16_t x_left;
  uint16_t x_right;
  uint16_t x_mid;

  // Get the number of blocks from the Pixy camera
  blocks = pixy.getBlocks(); 
  
  // If the image is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.blocks[0].x;
    x_right = x_left + (pixy.blocks[0].width);
    x_mid = ((x_right - x_left)/2) + x_left;

    // If the block is to the left, we want to move left.
    if(x_mid < (X_AXIS_MID - maxDeviation))
    {
      moveRobotLeft();
    }
    // If the block is to the right, we want to move right.
    else if(x_mid > (X_AXIS_MID + maxDeviation))
    {
      moveRobotRight();
    }
    // If the block is in the center, we want to move forward.
    else
    {
      moveRobotForward();
    }
  }
  delay(500);
} 

//**************************************************************************************************
// Name: moveRobotForward
//**************************************************************************************************
void moveRobotForward()
{
  Serial.write(moveFoward_ptr, 3);
  delay(210);
  Serial.write(moveFoward_ptr+3, 3);
}

//**************************************************************************************************
// Name: moveRobotLeft
//**************************************************************************************************
void moveRobotLeft()
{
  Serial.write(moveLeft_ptr, 3);
  delay(210);
  Serial.write(moveLeft_ptr+3, 3);
}

//**************************************************************************************************
// Name: moveRobotRight
//**************************************************************************************************
void moveRobotRight()
{
  Serial.write(moveRight_ptr, 3);
  delay(210);
  Serial.write(moveRight_ptr+3, 3);
}

//**************************************************************************************************
// Name: moveRobotBack
//**************************************************************************************************
void moveRobotBack()
{
  Serial.write(moveBack_ptr, 3);
  delay(210);
  Serial.write(moveBack_ptr+3, 3);
}
