#ifndef _COMMON_H_
#define _COMMON_H_

// Holds common enums and declarations

typedef struct {
  float x;
  float y;
  float z;
} Coordinate;

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
  STANDING = 0, BALANCED_STANDING, WALKING, BALANCED_WALKING
} RobotMode;

#endif