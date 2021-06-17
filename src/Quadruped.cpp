#include "Quadruped.h"


Quadruped::Quadruped(void) : leg1StepPlanner(LEG_1), leg1Kinematics(LEG_1),
                             leg2StepPlanner(LEG_2), leg2Kinematics(LEG_2),
                             leg3StepPlanner(LEG_3), leg3Kinematics(LEG_3),
                             leg4StepPlanner(LEG_4), leg4Kinematics(LEG_4)
{};

void Quadruped::_setMode(ROBOT_MODE robotMode) {
  _robotMode = robotMode;
}

void Quadruped::init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]) {

  _robotMode = STATIC_STANDING;

  leg1StepPlanner.init(inputZ);
  leg1Kinematics.init(inputX, inputY, inputZ, legMotors);

  leg2StepPlanner.init(inputZ);
  leg2Kinematics.init(inputX, inputY, inputZ, legMotors);

  leg3StepPlanner.init(inputZ);
  leg3Kinematics.init(inputX, inputY, inputZ, legMotors);

  leg4StepPlanner.init(inputZ);
  leg4Kinematics.init(inputX, inputY, inputZ, legMotors);
}

void Quadruped::walk(int16_t controlCoordinateX, int16_t controlCoordinateY) {
  if ((controlCoordinateX == 0) && (controlCoordinateY == 0) && _robotMode != STATIC_STANDING) {
    Serial.println("Stand pending");
    _setMode(STAND_PENDING);
  }
  if (((controlCoordinateX != 0) || (controlCoordinateY != 0)) && (_robotMode == STATIC_STANDING)){
    Serial.println("Walking");
    _setMode(WALKING);
    leg1StepPlanner.reset();
  }

  if ((_robotMode == WALKING) || (_robotMode == STAND_PENDING)) {
    if (leg1StepPlanner.footAtOrigin()) {
      Serial.println("At Origin");
      leg1StepPlanner.setStepEndpoint(controlCoordinateX, controlCoordinateY);
      if (_robotMode == STAND_PENDING) {
        while (!leg1StepPlanner.update(_robotMode))
        ;
        int16_t leg1InputX = leg1StepPlanner.dynamicFootPosition.x;
        int16_t leg1InputY = leg1StepPlanner.dynamicFootPosition.y;
        int16_t leg1InputZ = leg1StepPlanner.dynamicFootPosition.z;
        leg1Kinematics.setFootEndpoint(leg1InputX, leg1InputY, leg1InputZ);
        _setMode(STATIC_STANDING);
        Serial.println("Standing");
      }
    }
    if (leg1StepPlanner.update(_robotMode)) {
      Serial.println("Updated");
      int16_t leg1InputX = leg1StepPlanner.dynamicFootPosition.x;
      int16_t leg1InputY = leg1StepPlanner.dynamicFootPosition.y;
      int16_t leg1InputZ = leg1StepPlanner.dynamicFootPosition.z;

      leg1Kinematics.setFootEndpoint(leg1InputX, leg1InputY, leg1InputZ);
    }
  }

};