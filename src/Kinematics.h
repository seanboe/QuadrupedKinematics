// Kinematics.h
// Author: Sean Boerhout
// Copyright (C) 2021 Sean Boerhout


#ifndef _KINEMATICS_
#define _KINEMATICS_

#include <Arduino.h>

#include "Config.h"

#include <Ramp.h>

typedef struct {
  uint8_t controlPin;

  // Angle/calculation stuff
  int16_t angleDegrees;
  int16_t previousDegrees;     // previous degrees since last call to updateDynamicPositions()
  int16_t dynamicDegrees;

  // Calibration
  uint16_t calibOffset;         // This is an offset for calibration (to keep the motor accurate)
  uint16_t maxPos;
  uint16_t minPos;
  uint16_t applicationOffset;   // This is an offset for putting the calculated angles in contex.
                                // It is likely that the zero positions of the motors isn't where
                                // calculations assumes it to be, so you need an offset to make 
                                // sure that the angle is correct relative to the motor's zero.
} Motor;


class Kinematics {
  
  private:

    LegID _legID;
    
    uint16_t _indexOfMotor(LegID leg, MotorID motor);

    rampInt dynamicX;
    rampInt dynamicY;
    rampInt dynamicZ;

    Motor * _motors;  // list of ALL robot motors

  public:
  
    Kinematics();

    void init(LegID legID, int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]);

    // calculates all relevant motor angles (the angles of motors 2 & 3) to achieve vertical translation
    void solveFtShldrLength(float demandFtShldr, float *demandAngle2, float *demandAngle3);

    // calculates M2 angle offset to achieve a foward translation in the x direction
    void solveXMove(int16_t inputX, int16_t inputZ, float *demandAngle2, float *demandFtShldrLength);

    // calculates M1 angle offset to achieve y-axis movement
    void solveYMove(int16_t inputY, int16_t inputZ, float *demandAngle1, float *yPlaneZOutput);

    // general kinematics function; uses all positioning functions to place foot in 3d space
    void solveFootPosition(int16_t inputX, int16_t inputY, int16_t inputZ, int16_t *motor1AngleP, int16_t *motor2AngleP, int16_t *motor3AngleP);

    // This sets the foot endpoints for when you are updating the foot position dynamically. (Interpolating Foot position, not angle)
    void setFootEndpoint(int16_t inputX, int16_t inputY, int16_t inputZ);

    // This solves for the dynamic foot position using the interpolated (dynamic) values. 
    void updateDynamicFootPosition();

};

#endif