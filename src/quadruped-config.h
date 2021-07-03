#ifndef _QUADRUPED_CONFIG_
#define _QUADRUPED_CONFIG_

//******************* robot hardware constraints *******************

// limb length in millimeters
// The library assumes that your robot is symmetric.
#define LIMB_1  45
#define LIMB_2  125
#define LIMB_3  125

#define BODY_LENGTH 230   // Length of your robot front shoulder to the back shoulder. 

// shoulder to foot length constraints in mm - determined using max/min angles for motors 2 & 3
#define SHOULDER_FOOT_MAX 230
#define SHOULDER_FOOT_MIN 100


//****************** walking/gait setup *******************

#define TIME_TO_UPDATE          6    // The time between each update of the state machine + 1 i.e. this will update every 10 millis
#define GAIT_POSITION_INCREMENT 1     // The amount incremented and decremented to footXYDrop
#define INTER_STEP_PAUSE        250

// Uncomment whichever one you want, comment out the other. 
#define RIGHT_FOOTED                  
// #define LEFT_FOOTED

// #define STANDING_TROT

#define DRAW_BACK_AMPLITUDE_REDUCTION 4 // the draw back phase of the step also has an amplitude proportional to the arc amplitude. 

//******************* kinematics setup *******************
// Maximum motor speed; milliseconds per 180 degrees factor; NOT DEGREES PER MILLISECONDS I.E. SPEED (determined experimentally) this is 0.6 sec / 180 degrees (actual value is 0.52 sec)
#define MAX_SPEED_INVERSE 3.5
// #define MAX_SPEED_INVERSE   25


//******************* Static movement constraints *******************
#define YAW_MAXIMUM_ANGLE 20    // maximum yaw angle in degrees


// DON'T CHANGE BELOW HERE

// This is used to parse which motors are for which leg from the list of motors.
#define MOTORS_PER_LEG    3

// Do NOT change this! I'm only using this for documentation. This library is for QUADRUPEDS and NOTHING else!
#define ROBOT_LEG_COUNT   4

// leg numbering
typedef enum {
  LEG_1 = 1, LEG_2, LEG_3, LEG_4
} LegID;

typedef enum {
  M1 = 1, M2, M3
} MotorID;

typedef enum {
  STATIC_STANDING = 0, STAND_PENDING, WALKING
} ROBOT_MODE;

#endif