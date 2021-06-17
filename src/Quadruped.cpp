#include "Quadruped.h"


Quadruped::Quadruped(void) {};

void Quadruped::_setMode(ROBOT_MODE robotMode) {
  _robotMode = robotMode;
}

void Quadruped::init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]) {

  _robotMode = STATIC_STANDING;

  for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    LegID LEG = _enumFromIndex(leg);
    // LegID LEG = LEG_1;
    legStepPlanner[leg].init(LEG, inputZ);
    legKinematics[leg].init(LEG, inputX, inputY, inputZ, legMotors);
  }

  // leg1StepPlanner.init(LEG_1, inputZ);
  // leg1Kinematics.init(LEG_1, inputX, inputY, inputZ, legMotors);

  // leg2StepPlanner.init(LEG_2, inputZ);
  // leg2Kinematics.init(LEG_2, inputX, inputY, inputZ, legMotors);

  // leg3StepPlanner.init(LEG_3, inputZ);
  // leg3Kinematics.init(LEG_3, inputX, inputY, inputZ, legMotors);

  // leg4StepPlanner.init(LEG_4, inputZ);
  // leg4Kinematics.init(LEG_4, inputX, inputY, inputZ, legMotors);
}

void Quadruped::walk(int16_t controlCoordinateX, int16_t controlCoordinateY) {
  if ((controlCoordinateX == 0) && (controlCoordinateY == 0) && _robotMode != STATIC_STANDING) {
    _setMode(STAND_PENDING);
  }
  if (((controlCoordinateX != 0) || (controlCoordinateY != 0)) && (_robotMode == STATIC_STANDING)){
    _setMode(WALKING);
    for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++)
      legStepPlanner[leg].reset();
  }

  if ((_robotMode == WALKING) || (_robotMode == STAND_PENDING)) {
    for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
      if (legStepPlanner[leg].footAtOrigin()) {
        legStepPlanner[leg].setStepEndpoint(controlCoordinateX, controlCoordinateY);
        if (_robotMode == STAND_PENDING) {
            while (!legStepPlanner[leg].update(_robotMode))
              ;
            int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
            int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
            int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;
            legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);
            if (_enumFromIndex(leg) == LEG_4) _setMode(STATIC_STANDING);
            // _setMode(STATIC_STANDING);
        }
      }
      if (legStepPlanner[leg].update(_robotMode)) {
        int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
        int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
        int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;

        legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);
      }
    }
  }

};

LegID Quadruped::_enumFromIndex(int8_t index) {
  if (index == 0) return LEG_1;
  else if (index == 1) return LEG_2;
  else if (index == 2) return LEG_3;
  else if (index == 3) return LEG_4;
}