#ifndef _CONFIG_
#define _CONFIG_

// degrees to microseconds sale factor (determined experimentally, MUST be the same for every motor)
#define DEGREES_TO_MICROS 7.5

// Maximum motor speed; millis per 180 degrees factor; NOT DEGREES PER MILLIS I.E. SPEED (determined experimentally) this is 0.6 sec / 180 degrees (actual value is 0.52 sec)
#define MAX_SPEED_INVERSE 3.5

typedef enum {
  DEGREES, MILLIS
} unitType;


// leg numbering
#define LEG_1 1
#define LEG_2 2
#define LEG_3 3
#define LEG_4 4


// motor enums
typedef enum {
  M1, M2, M3
} motorID;


// default axis lengths (the 'safe' position for all the motors)
#define DEFAULT_X 0
#define DEFAULT_Y 45    // This is the same as LIMB_1; I want the foot to be directly under the shoulder ie straight, not under the bearing
#define DEFAULT_Z 177   // This is the foot-shoulder length when the leg makes a 45-45-90 triangle


//********* robot hardware constraints **********

// limb length in millimeters
#define LIMB_1  45
#define LIMB_2  125
#define LIMB_3  125


// shoulder to foot length constraints in mm - determined using max/min angles for motors 2 & 3
#define SHOULDER_FOOT_MAX 230
#define SHOULDER_FOOT_MIN 100


//********* Constants for legs 2 & 3 **********************
// motor angular offsets (0 position is different than what is assumed for triangle calcs)
#define M1_OFFSET 90
#define M2_OFFSET 90
#define M3_OFFSET 90


// motor angular limits (degrees)
#define M1_MAX 120
#define M1_MIN 45
#define M2_MAX 270
#define M2_MIN 0
#define M3_MAX 130     //130
#define M3_MIN 45

#endif