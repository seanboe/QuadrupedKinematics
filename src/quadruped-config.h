#ifndef _QUADRUPED_CONFIG_
#define _QUADRUPED_CONFIG_

// Note that EVERYTHING IS IN METRIC; the base unit is MILLIMETERS

////////////////////////////////////////////////////////////////////
//******************* robot hardware constraints *******************
////////////////////////////////////////////////////////////////////

// limb length in millimeters
// The library assumes that your robot is symmetric.
#define LIMB_1  45
#define LIMB_2  100
#define LIMB_3  100

#define BODY_LENGTH 230   // Length of your robot front shoulder to the back shoulder. 
#define BODY_WIDTH  170    // Width of your robot from the y-axis of rotation of one leg to the other leg,
                          // whether it be VIRTUAL OR REAL. This should be a point above the foot.

// shoulder to foot length constraints in mm - determined using max/min angles for motors 2 & 3
// For 125mm legs:
// #define SHOULDER_FOOT_MAX 200
// #define SHOULDER_FOOT_MIN 120

// For 100mm legs:
#define SHOULDER_FOOT_MAX    190
#define SHOULDER_FOOT_MIN    100

///////////////////////////////////////////////////////////
//****************** walking/gait setup *******************
///////////////////////////////////////////////////////////

#define GAIT_UPDATE_FREQUENCY   2    // The time between each update of the leg positions. Decreasing this increases the likelihood of an unfinished step or draw back. 
// #define STANDING_TROT

#define DRAW_BACK_AMPLITUDE_REDUCTION 4 // the draw back phase of the step also has an amplitude proportional to the arc amplitude. 

////////////////////////////////////////////////////////////////////
//******************* kinematics setup *******************
////////////////////////////////////////////////////////////////////

// Maximum motor speed; milliseconds per 180 degrees factor; NOT DEGREES PER MILLISECONDS I.E. SPEED (determined experimentally) this is 0.6 sec / 180 degrees (actual value is 0.52 sec)
// #define MAX_SPEED_INVERSE 3.5
#define MAX_SPEED_INVERSE   15

/////////////////////////////////////////////////////////////////////
//******************* Static movement constraints *******************
////////////////////////////////////////////////////////////////////
#define ROLL_MAXIMUM_ANGLE  20    // maximum roll angle in degrees
#define PITCH_MAXIMUM_ANGLE 15    // maximum pitch angle in degrees
#define YAW_MAXIMUM_ANGLE   20    // maximum yaw angle in degrees

#define PERCENT_LENGTH_TRANSLATION   10    // The percentage of the length a translation will be constrained to
#define PERCENT_WIDTH_TRANSLATION    10    // The percentage of the width a tranlation will be constrained to
#define MAX_HEIGHT                   180
#define MIN_HEIGHT                   110

//////////////////////////////////////////////////////////////////
//******************* IMU Feedback + PID gains *******************
//////////////////////////////////////////////////////////////////

#define CF_GYRO_BIAS_PITCH    0.98
#define CF_GYRO_BIAS_ROLL     0.98
#define PID_UPDATE_FREQUENCY  100     // Max 1000Hz


// /// Error test
// #define PITCH_KP   0      // 0.032 works well
// #define PITCH_KI   0        //0.26 works well
// #define PITCH_KD   0

// // Roll
// #define ROLL_KP    0   //0.03
// #define ROLL_KI    0  // 0.19
// #define ROLL_KD    0

// Pitch
#define PITCH_WALKING_KP   0.065
#define PITCH_WALKING_KI   0
#define PITCH_WALKING_KD   0.0003

// Roll
#define ROLL_WALKING_KP    0.065
#define ROLL_WALKING_KI    0
#define ROLL_WALKING_KD    0.0003


// Pitch
#define PITCH_STANDING_KP   0.01      // 0.1 works well
#define PITCH_STANDING_KI   1.5      // 1.3 works well
#define PITCH_STANDING_KD   0   // 0.0012 works well

// Roll
#define ROLL_STANDING_KP    0.013 //0.125
#define ROLL_STANDING_KI    1.8 //1.7
#define ROLL_STANDING_KD    0

// For 125mm legs
// // Pitch
// #define PITCH_KP   0.15
// #define PITCH_KI   2.8
// #define PITCH_KD   0

// // Roll
// #define ROLL_KP    0.4
// #define ROLL_KI    1.8
// #define ROLL_KD    0

#endif