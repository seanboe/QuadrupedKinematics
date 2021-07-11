#include "Quadruped.h"


Quadruped::Quadruped(void) {};

// void Quadruped::_setMode(RobotMode mode) {
//   _mode = mode;
// }

void Quadruped::init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]) {

  _mode = STANDING;

  for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    LegID LEG = _enumFromIndex(leg);
    legStepPlanner[leg].init(LEG, inputX, inputY, inputZ);
    legKinematics[leg].init(LEG, inputX, inputY, inputZ, legMotors);
  }
};

void Quadruped::loadGait(int16_t gaitParameters[], int16_t gaitSchedule[][ROBOT_LEG_COUNT]) {

  _gait.stepDistance = gaitParameters[STRIDE_LENGTH_INDEX];
  _gait.amplitude = gaitParameters[GAIT_AMPLITUDE_INDEX];
  _gait.drawBackReduction = gaitParameters[GAIT_DRAW_BACK_FACTOR_INDEX];
  _gait.stepDuration = gaitParameters[STEP_DURATION_INDEX];
  _gait.stepCount = gaitParameters[STEP_COUNT_INDEX];
  _gait.pauseDuration = gaitParameters[PAUSE_DURATION_INDEX];

  for (int16_t x = 0; x < _gait.stepCount; x++ ) {
    for (int16_t i = 0; i < ROBOT_LEG_COUNT; i++)
      _gaitSchedule[x][i] = gaitSchedule[x][i];
  }
}

void Quadruped::walk(int16_t controlCoordinateX, int16_t controlCoordinateY) {

  if (_mode == STANDING) {
    _mode = WALKING;
    _firstStep = true;
    _previousStepUpdate = millis();
    _justUpdatedWalk = false;
    _currentGaitScheduleIndex = 0;
  }

  if (_mode == WALKING) {
    if (((millis() - _previousStepUpdate) % _gait.stepDuration == 0) && _justUpdatedWalk == false) {

      Serial.println("hereeee");

      int16_t stepDistance = 0;
      int16_t drawBackDistance = 0;

      // Weird stuff here; fix later. 
      if (_firstStep) {
        Serial.println("first step");
        if (_gait.stepCount == 2) {
          stepDistance = _gait.stepDistance / 2;
          drawBackDistance = _gait.stepDistance / 2;
        }
        else if (_gait.stepCount == 4) {
          if (_currentGaitScheduleIndex == _gait.stepCount - 1)  {
            drawBackDistance = _gait.stepDistance / 3;
            stepDistance = _gait.stepDistance;
            Serial.println("herelksdfjlsdkfjlksdjf");
          }
          else {
            stepDistance = (2*(_currentGaitScheduleIndex) + 1) * _gait.stepDistance / 6;
            drawBackDistance = _gait.stepDistance / 6;
          }
        }
      }
      else {
        drawBackDistance = _gait.stepDistance / 3;
        stepDistance = _gait.stepDistance;
      }

      for (int16_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
        switch (_gaitSchedule[_currentGaitScheduleIndex][leg]) {
          case TAKE_STEP:
            legStepPlanner[leg].calculateStep(controlCoordinateX, controlCoordinateY, _gait.stepDuration, stepDistance);
            break;
          case DRAW_BACK:
                                  if (legStepPlanner[leg].finishedAction()) Serial.println("We're done");

            legStepPlanner[leg].calculateDrawBack(controlCoordinateX, controlCoordinateY, _gait.stepDuration, drawBackDistance);
            break;
          case PAUSE: break;
        }
      }

      _currentGaitScheduleIndex++;
      if (_currentGaitScheduleIndex == _gait.stepCount) {
        _currentGaitScheduleIndex = 0;
        _firstStep = false;
      }
      
      _justUpdatedWalk = true;
    }

    if ((millis() - _previousStepUpdate) % _gait.stepDuration != 0) _justUpdatedWalk = false;

  }

  updateLegPositions();

  // if (legStepPlanner[0].update()) {
  //   int16_t inputX = legStepPlanner[0].dynamicFootPosition.x;
  //   int16_t inputY = legStepPlanner[0].dynamicFootPosition.y;
  //   int16_t inputZ = legStepPlanner[0].dynamicFootPosition.z;

  //   legKinematics[0].setFootEndpoint(inputX, inputY, inputZ);

  // }

};

void Quadruped::updateLegPositions() {
  for (int16_t leg = 0; leg < 3; leg++) {
    if (legStepPlanner[leg].update()) {
      int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
      int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
      int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;

      legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);

    }
  }
}


  // if (((controlCoordinateX != 0) || (controlCoordinateY != 0)) && (_robotMode == STATIC_STANDING)){
  //   _setMode(WALKING);
  //   for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++)
  //     legStepPlanner[leg].reset();
  // }

  // if ((_mode == WALKING) || (_robotMode == STAND_PENDING)) {
  //   for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
  //     if (legStepPlanner[leg].footAtOrigin()) {
  //       legStepPlanner[leg].setStepEndpoint(controlCoordinateX, controlCoordinateY, _robotMode, computeYaw(yawInput));
  //       if (_robotMode == STAND_PENDING) {
  //           while (!legStepPlanner[leg].update(_robotMode))
  //             ;
  //           int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
  //           int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
  //           int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;
  //           legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);
  //           if (_enumFromIndex(leg) == LEG_4) _setMode(STATIC_STANDING);
  //       }
  //     }
  //     if (legStepPlanner[leg].update(_robotMode)) {
  //       int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
  //       int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
  //       int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;

  //       legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);
  //     }
  //   }
  // }







// int16_t Quadruped::computeYaw(int16_t yawAngle) {
//   // Map the controller input to a yaw angle. It is assumed that 0 degrees is when the body of the robot is straight
//   // forwards, with degrees increasing as you approach the rear of the robot. 
//   if (abs(yawAngle) > YAW_MAXIMUM_ANGLE) {
//     if (yawAngle > 0) yawAngle = YAW_MAXIMUM_ANGLE;
//     if (yawAngle < 0) yawAngle = -1 * YAW_MAXIMUM_ANGLE;
//   }

//   // Serial.println(lrint((BODY_LENGTH / 2) * atan((yawAngle * PI)/180)));

//   return lrint((BODY_LENGTH / 2) * atan((yawAngle * PI)/180));

// };


LegID Quadruped::_enumFromIndex(int8_t index) {
  if (index == 0) return LEG_1;
  else if (index == 1) return LEG_2;
  else if (index == 2) return LEG_3;
  else if (index == 3) return LEG_4;
  else return LEG_1;
};



// #elif defined(STANDING_TROT)

//   if (((controlCoordinateX != 0) || (controlCoordinateY != 0)) && (_robotMode == STATIC_STANDING)){
//     _setMode(WALKING);
//     for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++)
//       legStepPlanner[leg].reset();
//   }

//   if ((_robotMode == WALKING)) {
//     for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
//       if (legStepPlanner[leg].footAtOrigin()) {
//         legStepPlanner[leg].setStepEndpoint(controlCoordinateX, controlCoordinateY, _robotMode, computeYaw(yawInput));
//       }
//       if (legStepPlanner[leg].update(_robotMode)) {
//         int16_t inputX = legStepPlanner[leg].dynamicFootPosition.x;
//         int16_t inputY = legStepPlanner[leg].dynamicFootPosition.y;
//         int16_t inputZ = legStepPlanner[leg].dynamicFootPosition.z;

//         legKinematics[leg].setFootEndpoint(inputX, inputY, inputZ);
//       }
//     }
//   }


// #endif