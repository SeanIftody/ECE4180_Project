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
#define X_AXIS_MID 160  // Middle point in the x-axis
#define Y_AXIS_MID 100  // Middle point in the y-axis

#define X_AXIS_R_FOOT 200 // Point around right foot.

// FSM States for robot
#define ROBO_FSM_FIND     0 // State for finding the ball
#define ROBO_FSM_WALK     1 // State for walking towards the ball
#define ROBO_FSM_POSITION 2 // State for aligning robot with ball
#define ROBO_FSM_KICK     3 // State for kicking the ball

//**************************************************************************************************
// VARIABLES
//**************************************************************************************************
Pixy pixy; // Create a Pixy camera object

double maxDeviation = 0.75 * (X_AXIS_MID/2); // Determines the max deviation from the center
double maxDevFoot   = 0.97 * (X_AXIS_R_FOOT/2); // Determines the max deviation from the r foot

uint8_t fsmState; // Defines state in FSM which controls the robot.

//**************************************************************************************************
// CONSTANTS
//**************************************************************************************************
// Instructions to move forward, left, right, and back.
const uint8_t  moveFoward[6]  = {0xAA, 0x01, 0x0E,
                                 0xAA, 0x00, 0x1E};
const uint8_t* moveFoward_ptr = moveFoward;
const uint8_t  moveLeft[6]    = {0xAA, 0x04, 0x5E,
                                 0xAA, 0x00, 0x1E};
const uint8_t* moveLeft_ptr   = moveLeft;
const uint8_t  moveRight[6]   = {0xAA, 0x08, 0x1E,
                                 0xAA, 0x00, 0x1E};
const uint8_t* moveRight_ptr  = moveRight;
const uint8_t  moveBack[6]    = {0xAA, 0x02, 0x7E, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* moveBack_ptr   = moveBack;

// Slide to left and right
const uint8_t  slideLeft[12]  = {0xAA, 0x24, 0x5E, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* slideLeft_ptr  = slideLeft;
const uint8_t  slideRight[12] = {0xAA, 0x28, 0x1E, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* slideRight_ptr = slideRight;

// Look down and up
const uint8_t  lookDown[12]   = {0xAA, 0x40, 0x1E, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* lookDown_ptr   = lookDown;
const uint8_t  lookUp[12]     = {0xAA, 0x10, 0x1E, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* lookUp_ptr     = lookUp;


// Kick action
const uint8_t  kickAction[12] = {0xAA, 0x00, 0x0F, 
                                 0xAA, 0x00, 0x1E};
const uint8_t* kickAction_ptr = kickAction;

//**************************************************************************************************
// FUNCTION PROTOTYPES
//**************************************************************************************************
// Robot states
void walkTowardsBall();
void positionRobot();
void performKick();

// Robot actions
void moveRobotForward();
void moveRobotLeft();
void moveRobotRight();
void moveRobotBack();
void slideRobotLeft();
void slideRobotRight();
void robotLookUp();
void robotLookDown();
void robotKick();

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

  // Initialize FSM state
  //************************************************************************************************
  // Due to time constrains, the robot will assume that it already has seen the ball. For future
  // work, the robot should be able to find the ball, and should incorperate Hough Transform to
  // find the shape of the ball as well.
  //************************************************************************************************
  fsmState = ROBO_FSM_WALK;
}

//**************************************************************************************************
// Name: loop
//**************************************************************************************************
void loop()
{
  // Run state machine for every loop
  switch(fsmState)
  {
    // Find the ball
    case ROBO_FSM_FIND:
      // To be done. When completed, make sure to uncomment the break.
      //break;

    // Make the robot walk towards the ball.
    case ROBO_FSM_WALK:
      walkTowardsBall();
      break;

    // Make the robot position the ball.
    case ROBO_FSM_POSITION:
      positionRobot();
      break;

    // Have the robot kick the ball.
    case ROBO_FSM_KICK:
      performKick();
      break;

    // Default case
    default:
      break;
  }
}

//**************************************************************************************************
// Name: walkTowardsBall
//**************************************************************************************************
void walkTowardsBall()
{
  uint16_t blocks; // Number of blocks found by Pixy camera
  uint16_t x_left;
  uint16_t x_right;
  uint16_t x_mid;
  uint16_t y_axis_point;

  // Get the number of blocks from the Pixy camera
  blocks = pixy.getBlocks(); 
  
  // If the image is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.blocks[0].x;
    x_right = x_left + (pixy.blocks[0].width);
    x_mid = ((x_right - x_left)/2) + x_left;

    y_axis_point = pixy.blocks[0].y;

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
    else
    {
      // If the ball is not close, walk towards the ball
      if( y_axis_point < 183 )
      {
        moveRobotForward();
      }
      // If the ball is close enough, then it is time to switch to the next state.
      else
      {
        fsmState = ROBO_FSM_POSITION;
      }
    }
  }
  else
  {
    //**********************************************************************************************
    // If the camera(s) can not detect the ball, then the robot will have to switch to the state
    // in which it will search for the ball. Because this is not implemented however, it should
    // be done in the future.
    //**********************************************************************************************
    fsmState = ROBO_FSM_FIND;
  }
  delay(250);
}

//**************************************************************************************************
// Name: positionRobot
//**************************************************************************************************
void positionRobot()
{
  uint16_t blocks;
  uint16_t x_left;
  uint16_t x_right;
  uint16_t x_mid;
  uint16_t y_axis_point;

  //************************************************************************************************
  // To position the robot, the robot should be looking downwards so it knows where the ball is
  // located with respect to its right foot.
  //************************************************************************************************
  robotLookDown();
  delay(1000);

  // Get the image details.
  blocks = pixy.getBlocks(); 
  
  // As soon as the infromation is returned, make the robot look up again.
  delay(1000);
  robotLookUp();
  delay(1000);

  // If the image is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.blocks[0].x;
    x_right = x_left + (pixy.blocks[0].width);
    x_mid = ((x_right - x_left)/2) + x_left;

    y_axis_point = pixy.blocks[0].y;

    // If the ball is close enough, but not aligned, shuffle
    if(x_mid < X_AXIS_R_FOOT)
    {
      slideRobotLeft();
    }
//    else if(x_mid > (X_AXIS_R_FOOT + maxDevFoot))
//    {
//      slideRobotRight();
//    }
    // If the block is not close enough, move forward.
    else if(y_axis_point < 90)
    {
      moveRobotForward();
    }
    // If the block is to the right, we want to move right.
    else
    {
      fsmState = ROBO_FSM_KICK;
    }
  }
  else
  {
    //**********************************************************************************************
    // If the robot can not find the ball (although it is assumed it should be right infront of it),
    // then the robot should switch to the find ball mode. For now, however, it will be put into
    // the walk mode.
    //**********************************************************************************************
    fsmState = ROBO_FSM_FIND;
  }
}

//**************************************************************************************************
// Name: performKick
//**************************************************************************************************
void performKick()
{
  robotKick();
  fsmState = ROBO_FSM_FIND;
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


//**************************************************************************************************
// Name: slideRobotLeft
//**************************************************************************************************
void slideRobotLeft()
{
  Serial.write(slideLeft_ptr, 3);
  delay(210);
  Serial.write(slideLeft_ptr+3, 3);
}

//**************************************************************************************************
// Name: slideRobotRight
//**************************************************************************************************
void slideRobotRight()
{
  Serial.write(slideRight_ptr, 3);
  delay(210);
  Serial.write(slideRight_ptr+3, 3);
}

//**************************************************************************************************
// Name: robotLookUp
//**************************************************************************************************
void robotLookUp()
{
  Serial.write(lookUp_ptr, 3);
  delay(210);
  Serial.write(lookUp_ptr+3, 3);
}

//**************************************************************************************************
// Name: robotLookDown
//**************************************************************************************************
void robotLookDown()
{
  Serial.write(lookDown_ptr, 3);
  delay(210);
  Serial.write(lookDown_ptr+3, 3);
}

//**************************************************************************************************
// Name: robotKick
//**************************************************************************************************
void robotKick()
{
  Serial.write(kickAction_ptr, 3);
  delay(210);
  Serial.write(kickAction_ptr+3, 3);
}
