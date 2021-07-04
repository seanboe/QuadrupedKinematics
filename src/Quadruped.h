#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include "stepPlanner.h"
#include "Kinematics.h"
#include "quadruped-config.h"

class Quadruped {
  public:
    Quadruped();

    void init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]);

    void walk(int16_t controlCoordinateX, int16_t controlCoordinateY, int16_t yawInput);
    void update(int16_t controlCoordinateX, int16_t controlCoordinateY);

    int16_t computeYaw(int16_t yawAngle);

  private:

    void _setMode(ROBOT_MODE robotMode);
    LegID _enumFromIndex(int8_t index);

    StepPlanner legStepPlanner[ROBOT_LEG_COUNT];
    Kinematics  legKinematics[ROBOT_LEG_COUNT];

    RobotMode _mode;

};


#endif