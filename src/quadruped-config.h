#ifndef _QUADRUPED_CONFIG_
#define _QUADRUPED_CONFIG_

// Note that EVERYTHING IS IN METRIC; the base unit is MILLIMETERS

////////////////////////////////////////////////////////////////////
//******************* robot hardware constraints *******************
////////////////////////////////////////////////////////////////////

// limb length in millimeters
// The library assumes that your robot is symmetric.
#define LIMB_1  45
#define LIMB_2  125
#define LIMB_3  125

#define BODY_LENGTH 230   // Length of your robot front shoulder to the back shoulder. 
#define BODY_WIDTH  84

// shoulder to foot length constraints in mm - determined using max/min angles for motors 2 & 3
#define SHOULDER_FOOT_MAX 230
#define SHOULDER_FOOT_MIN 100

///////////////////////////////////////////////////////////
//****************** walking/gait setup *******************
///////////////////////////////////////////////////////////

#define GAIT_UPDATE_FREQUENCY   2    // The time between each update of the leg positions. Decreasing this increases the likelihood of an unfinished step or draw back. 
// #define STANDING_TROT

#define MAX_STEPS   4

#define DRAW_BACK_AMPLITUDE_REDUCTION 4 // the draw back phase of the step also has an amplitude proportional to the arc amplitude. 

////////////////////////////////////////////////////////////////////
//******************* kinematics setup *******************
////////////////////////////////////////////////////////////////////

// Maximum motor speed; milliseconds per 180 degrees factor; NOT DEGREES PER MILLISECONDS I.E. SPEED (determined experimentally) this is 0.6 sec / 180 degrees (actual value is 0.52 sec)
#define MAX_SPEED_INVERSE 3.5
// #define MAX_SPEED_INVERSE   25

/////////////////////////////////////////////////////////////////////
//******************* Static movement constraints *******************
////////////////////////////////////////////////////////////////////
#define ROLL_MAXIMUM_ANGLE  20    // maximum roll angle in degrees
#define YAW_MAXIMUM_ANGLE   20    // maximum yaw angle in degrees
#define PITCH_MAXIMUM_ANGLE 20    // maximum pitch angle in degrees

#define PERCENT_LENGTH_TRANSLATION   50    // The percentage of the length a translation will be constrained to
#define PERCENT_WIDTH_TRANSLATION    50    // The percentage of the width a tranlation will be constrained to

//////////////////////////////////////////////////////////////////
//******************* IMU Feedback + PID gains *******************
//////////////////////////////////////////////////////////////////

#define LPF_SMOOTHING_FACTOR  0.1     // The smoothing factor for the low pass filter on the imu
#define IMU_FLIPPED                   // Uncomment this if your IMU is flipped 180 degrees (upside down)

#endif