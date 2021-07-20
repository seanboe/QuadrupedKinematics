#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include "StepPlanner.h"
#include "Kinematics.h"
#include "quadruped-config.h"
#include "common.h"
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

    void giveIMUFeedback(float accelX, float accelY, float accelZ);
    void getRollPitch(float *roll, float *pitch);

    StepPlanner legStepPlanner[ROBOT_LEG_COUNT];
    Kinematics  legKinematics[ROBOT_LEG_COUNT];

    void computeStaticMovement(int16_t offsetX, int16_t offsetY, int16_t offsetZ, int16_t rollAngle, int16_t pitchAngle, int16_t yawAngle);
    void compute(RobotMode desiredMode, int16_t inputX, int16_t inputY, int16_t input, int16_t rotationX, int16_t rotationY, int16_t rotationZ);

  private:

    LegID _enumFromIndex(int8_t index);

    // Global Parameters
    RobotMode _mode;
    Coordinate _footPositions[ROBOT_LEG_COUNT];
    Coordinate _originFootPosition;
    int16_t RobotHeight;

    // Balanced Standing
    Coordinate _IMUData;
    Coordinate _filteredIMUData;
    bool _haveIMUFeedback;    

    // Walking
    Gait _gait;
    int16_t _gaitSchedule[MAX_STEPS][ROBOT_LEG_COUNT];
    int16_t _currentGaitScheduleIndex;

    unsigned long _previousStepUpdate;
    bool _justUpdatedWalk;
    bool _firstStep;

};


#endif