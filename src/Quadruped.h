#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include "stepPlanner.h"
#include "Kinematics.h"
#include "Config.h"

class Quadruped {
  public:
    Quadruped();

    void init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]);

    void walk(int16_t controlCoordinateX, int16_t controlCoordinateY);
    bool justSetEndpoint = false;

  private:

    StepPlanner leg1StepPlanner;
    Kinematics  leg1Kinematics;

    StepPlanner leg2StepPlanner;
    Kinematics  leg2Kinematics;

    StepPlanner leg3StepPlanner;
    Kinematics  leg3Kinematics;

    StepPlanner leg4StepPlanner;
    Kinematics  leg4Kinematics;

    ROBOT_MODE _robotMode;

};


#endif