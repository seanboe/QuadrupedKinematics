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
    StepPlanner();
    void init(LegID legID, int16_t offsetX, int16_t offsetY, int16_t robotHeight);
    void setGait(GaitType gaitType);
    bool update(ROBOT_MODE robotMode);
    void setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, ROBOT_MODE robotMode);
    int16_t getStepHeight(int16_t footXYDropL, LegMode movementType);
    bool footAtOrigin();
    void reset();

    Coordinate dynamicFootPosition;

  private: 

    bool _setFirstStep(ROBOT_MODE robotMode);

    LegID _legID; 
    int16_t _robotHeight;
    int16_t _offsetX;
    int16_t _offsetY;

    rampFloat footPosX;
    rampFloat footPosY;

    bool _wasAtOrigin;    

    Gait _gaits[NUMBER_OF_GAITS];

    int16_t _footXYDrop; // the position of the foot on the x/y plane. It moves underneath the foot.

    long _previousUpdateTime;

    Coordinate _stepEndpoint;

    LegMode _legMode; 
    GaitType _gaitType;


};

#endif

/*
Be aware that I made a mistake organizing the axis. Looking down the robot, moving the feet forwards is a movement along the x axis as defined in class Kinematics. Y is side to side.
This, however, isn't intuitive; you'd normally expect the other way around, which means that x and y must be flipped. currently, everything is aligned to the standard in Kinematics,
but this means that the data must be flipped for users (x is y and y is x), which is done in setStepEndpoint.
*/