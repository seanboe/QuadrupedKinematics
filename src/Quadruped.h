#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include "StepPlanner.h"
#include "Kinematics.h"
#include "quadruped-config.h"
#include "gait-scheduler-codes.h"


typedef struct {
  int16_t stepDistance;
  int16_t amplitude;
  int16_t drawBackReduction;
  int16_t stepDuration;
  int16_t stepCount;
  int16_t pauseDuration;
} Gait;

class Quadruped {
  public:
    Quadruped();

    void init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]);
    void loadGait(int16_t gaitParameters[], int16_t gaitSchedule[][ROBOT_LEG_COUNT]);

    void walk(int16_t controlCoordinateX, int16_t controlCoordinateY);
    void updateLegs();


    int16_t computeYaw(int16_t yawAngle);

    StepPlanner legStepPlanner[ROBOT_LEG_COUNT];
    Kinematics  legKinematics[ROBOT_LEG_COUNT];


  private:

    // void _setMode(ROBOT_MODE robotMode);
    LegID _enumFromIndex(int8_t index);

    RobotMode _mode;

    Gait _gait;
    int16_t _gaitSchedule[][ROBOT_LEG_COUNT];
    int16_t _currentGaitScheduleIndex;

    unsigned long _previousStepUpdate;
    bool _justUpdatedWalk;
    bool _firstStep;

};


#endif