#ifndef GAIT_PLANNER_H
#define GAIT_PLANNER_H

#include <Arduino.h>
#include "quadruped-config.h"
#include <Ramp.h>


typedef struct {
  float amplitude;
  float periodHalf;    // It is assumed that the gait arc is symmetrical across the y axis; half the frequency is the amount it goes forward and backwards
  float timeToUpdate;  // The time between updates
} Gait;

typedef struct {
  float x;
  float y;
  float z;
} Coordinate;

typedef enum {
  FIRST_STEP_ARC, FIRST_STEP_DRAW_BACK,
  ACTIVE_WALKING_ARC, ACTIVE_WALKING_DRAW_BACK
} StepStage;

class StepPlanner {

  public:
    StepPlanner();
    void init(LegID legID, int16_t offsetX, int16_t offsetY, int16_t robotHeight);
    void setGait(GaitType gaitType);
    bool update();
    bool setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, RobotMode robotMode, int16_t yawOffset);
    int16_t getStepHeight(int16_t footXYDropL);
    bool footAtOrigin();
    void reset();

    Coordinate dynamicFootPosition;

  private: 

    void _getWalkEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, int16_t* stepEndpointX, int16_t* stepEndpointY)

    LegID _legID; 
    int16_t _robotHeight;
    int16_t _offsetX;
    int16_t _offsetY;

    rampFloat footPosX;
    rampFloat footPosY;
    rampFloat footDrop

    bool _wasAtOrigin;    

    long _previousUpdateTime;

    Coordinate _stepEndpoint;

    RobotMode _mode; 
    StepStage _walkingStage;

    Gait _gait;


};

#endif

/*
Be aware that I made a mistake organizing the axis. Looking down the robot, moving the feet forwards is a movement along the x axis as defined in class Kinematics. Y is side to side.
This, however, isn't intuitive; you'd normally expect the other way around, which means that x and y must be flipped. currently, everything is aligned to the standard in Kinematics,
but this means that the data must be flipped for users (x is y and y is x), which is done in setStepEndpoint.
*/