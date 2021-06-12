#ifndef _CONFIG_
#define _CONFIG_

// degrees to microseconds sale factor (determined experimentally, MUST be the same for every motor)
#define DEGREES_TO_MICROS 7.5

// Maximum motor speed; milliseconds per 180 degrees factor; NOT DEGREES PER MILLISECONDS I.E. SPEED (determined experimentally) this is 0.6 sec / 180 degrees (actual value is 0.52 sec)
// #define MAX_SPEED_INVERSE 3.5
#define MAX_SPEED_INVERSE   25

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


//********* robot hardware constraints **********

// limb length in millimeters
// The library assumes that your robot is symmetric.
#define LIMB_1  45
#define LIMB_2  125
#define LIMB_3  125


// shoulder to foot length constraints in mm - determined using max/min angles for motors 2 & 3
#define SHOULDER_FOOT_MAX 230
#define SHOULDER_FOOT_MIN 100

#endif