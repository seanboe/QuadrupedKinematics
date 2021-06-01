#ifndef _KINEMATICS_
#define _KINEMATICS_

#include <Arduino.h>

#include "Config.h"

#include <Ramp.h>

typedef struct {
  uint16_t angleDegrees;
  uint16_t previousDegrees;     // previous degrees since last call to updateDynamicPositions()
  uint16_t angleMicros;
  uint16_t dynamicDegrees;
  uint16_t dynamicMicros;
  uint16_t calibOffset;
} motor;


class Kinematics {
  
  private:

    uint8_t _legID;

    uint16_t _degreesToMicros(uint8_t inputDegrees, uint8_t calibOffset);

    float _applyConstraints(uint8_t motor, float demandAngle);

    rampInt dynamicX;
    rampInt dynamicY;
    rampInt dynamicZ;

  public:

    motor motor1;
    motor motor2;
    motor motor3;

    // Kinematics(uint8_t legID, uint16_t motor1CalibOffset, uint16_t motor1StartPos, uint16_t motor2CalibOffset, uint16_t motor2StartPos, uint16_t motor3CalibOffset, uint16_t motor3StartPos);
    Kinematics(uint8_t legID, int16_t inputX, int16_t inputY, int16_t inputZ, uint16_t motor1CalibOffset, uint16_t motor2CalibOffset, uint16_t motor3CalibOffset);

    // calculates all relevant motor angles (the angles of motors 2 & 3) to achieve vertical translation
    void solveFtShldrLength(float demandFtShldr, float *demandAngle2, float *demandAngle3);

    // calculates M2 angle offset to achieve a foward translation in the x direction
    void solveXMove(int16_t inputX, int16_t inputZ, float *demandAngle2, float *demandFtShldrLength);

    // calculates M1 angle offset to achieve y-axis movement
    void solveYMove(int16_t inputY, int16_t inputZ, float *demandAngle1, float *yPlaneZOutput);

    // general kinematics function; uses all positioning functions to place foot in 3d space
    void solveFootPosition(int16_t inputX, int16_t inputY, int16_t inputZ, uint16_t *motor1AngleP, uint16_t *motor2AngleP, uint16_t *motor3AngleP);

    // This sets the foot endpoints for when you are updating the foot position dynamically. (Interpolating Foot position, not angle)
    void setFootEndpoint(int16_t inputX, int16_t inputY, int16_t inputZ);

    // This solves for the dynamic foot position using the interpolated (dynamic) values. 
    void updateDynamicFootPosition();

};

#endif