#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include <PID_v1.h>
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

    void init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[], bool willProvideIMUFeedback = false);
    void setMode(RobotMode mode);

    void loadGait(int16_t gaitParameters[], int16_t gaitSchedule[][ROBOT_LEG_COUNT]);
    void walk(int16_t controlCoordinateX, int16_t controlCoordinateY);

    void giveIMUFeedback(double accelX, double accelY, double accelZ);
    void getPitchRoll(double *roll, double *pitch);
    void setBalanceOrientation(int16_t rollEndpoint, int16_t pitchEndpoint);

    StepPlanner legStepPlanner[ROBOT_LEG_COUNT];
    Kinematics  legKinematics[ROBOT_LEG_COUNT];

    void computeStaticMovement(int16_t offsetX, int16_t offsetY, int16_t offsetZ, int16_t rollAngle, int16_t pitchAngle, int16_t yawAngle);
    void compute(int16_t inputX, int16_t inputY, int16_t inputZ, int16_t rotationX, int16_t rotationY, int16_t rotationZ);


  private:

    LegID _enumFromIndex(int8_t index);

    // Global Parameters
    RobotMode _mode;
    Coordinate _footPositions[ROBOT_LEG_COUNT];
    Coordinate _originFootPosition;             // For static movement, this is what all movements are based on

    // Balanced Standing
    Coordinate _IMUData;
    Coordinate _filteredIMUData;
    bool _willProvideIMUFeedback;    
    // Pitch
    PID pitchPID;
    double _measuredPitchAngle;
    double _outputPitchAngle;
    double _pitchSetpoint;
    // Roll
    PID rollPID;
    double _measuredRollAngle;
    double _outputRollAngle;
    double _rollSetpoint;
    
    #ifdef WANTS_FIFO_BUFFER
    double FIFOPIDBuffer[FIFO_BUFFER_SIZE];
    #endif

    // Walking
    Gait _gait;
    int16_t _gaitSchedule[MAX_STEPS][ROBOT_LEG_COUNT];
    int16_t _currentGaitScheduleIndex;

    unsigned long _previousStepUpdate;
    bool _justUpdatedWalk;
    bool _firstStep;

};


#endif