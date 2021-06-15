#ifndef GAIT_PLANNER_H
#define GAIT_PLANNER_H

#include <Arduino.h>
#include "Config.h"
#include <Ramp.h>

#define DEFAULT_GAIT  TROT

typedef enum {
  TROT = 0
} GaitType;
#define NUMBER_OF_GAITS 1

typedef struct {
  float amplitude;
  float periodHalf;    // It is assumed that the gait arc is symmetrical across the y axis; half the frequency is the amount it goes forward and backwards
} Gait;

typedef struct {
  float x;
  float y;
  float z;
} Coordinate;

typedef enum {
  FIRST_STEP_ARC, FIRST_STEP_DRAW_BACK,
  ACTIVE_WALKING_ARC, ACTIVE_WALKING_DRAW_BACK,
  STANDING
} LegMode;

class StepPlanner {

  public:
    StepPlanner(LegID legID);

    void init();

    void setGait(GaitType gaitType);

    void update(ROBOT_MODE robotMode);
    void setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, ROBOT_MODE robotMode);

    int16_t getStepHeight(int16_t footXYDropL, LegMode movementType);

    bool footAtOrigin();

    Coordinate dynamicFootPosition;

  private: 

    rampInt footPosX;
    rampInt footPosY;

    LegID _legID;     

    Gait _gaits[NUMBER_OF_GAITS];

    int16_t _footXYDrop; // the position of the foot on the x/y plane. It moves underneath the foot.

    long _previousUpdateTime;

    Coordinate _stepEndpoint;

    LegMode _legMode; 
    GaitType _gaitType;

    int16_t _robotHeight;

};

#endif