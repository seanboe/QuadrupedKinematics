#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include <PID_v1.h>
#include <simpleFusion.h>
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
  int16_t total_actions;
} Gait;

class Quadruped {
  public:
    Quadruped();

    void init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]);
    void setMode(RobotMode mode);

    void loadGait(int16_t gaitParameters[], int16_t gaitSchedule[][ROBOT_LEG_COUNT]);
    void walk(int16_t controlCoordinateX, int16_t controlCoordinateY, Coordinate outputFootPositions[ROBOT_LEG_COUNT]);

    void giveIMUFeedback(ThreeAxis &accelData, ThreeAxis &gyroData);
    void getPitchRoll(double *pitch, double *roll);
    void setBalanceOrientation(int16_t rollEndpoint, int16_t pitchEndpoint);
    bool wantsIMUUpdate();

    StepPlanner legStepPlanner[ROBOT_LEG_COUNT];
    Kinematics  legKinematics[ROBOT_LEG_COUNT];

    void computeStaticMovement(Coordinate translationOffsets, Coordinate rotationAngles, Coordinate outputFootPositions[ROBOT_LEG_COUNT]);                                 // x = roll, y = pitch, z = yaw (the axis each rotation is about)
    void compute(int16_t inputX = 0, int16_t inputY = 0, int16_t inputZ = 0, int16_t rotationX = 0, int16_t rotationY = 0, int16_t rotationZ = 0);


  private:

    LegID _enumFromIndex(int8_t index);
    
    // Global Parameters
    RobotMode _mode;                             // Actual Mode
    RobotMode _userDesiredMode;                  // The mode requested by the user
    Coordinate _footPositions[ROBOT_LEG_COUNT];
    Coordinate _originFootPosition;             // For static movement, this is what all movements are based on

    // Balanced Standing
    ThreeAxis _accelData;
    ThreeAxis _gyroData;
    SimpleFusion ImuFuser;

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

    // Walking
    Gait _gait;
    int16_t _gaitSchedule[MAX_STEPS][ROBOT_LEG_COUNT];      // Make sure that max_steps is large enough!
    int16_t _currentGaitScheduleIndex;

    unsigned long _previousStepUpdate;
    bool _justUpdatedWalk;
    bool _firstStep;
    bool _shouldShift;

    /// For PID Testing
    uint16_t pidLoopCount = 0;
    double pitchROC = 0;
    double rollROC = 0;

    double pitchError = 0;
    double rollError = 0;

    double previousMeasuredPitchAngle = 0;
    double previousMeasuredRollAngle = 0;

    long thispreviousTime = 0;

};


#endif