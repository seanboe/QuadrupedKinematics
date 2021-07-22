#include "Quadruped.h"


Quadruped::Quadruped(void) {};

/*!
 *    @brief Initializes robot parameters
 *    @param inputX Initial X input
 *    @param inputY Initial Y input
 *    @param inputZ Initial Z input
 *    @param legMotors  Array of Motor structs
*/
void Quadruped::init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]) {

  _mode = STANDING;

  for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    LegID LEG = _enumFromIndex(leg);
    legStepPlanner[leg].init(LEG, inputX, inputY, inputZ);
    legKinematics[leg].init(LEG, inputX, inputY, inputZ, legMotors);
  }

  for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    _footPositions[leg].x = inputX;
    _footPositions[leg].y = inputY;
    _footPositions[leg].z = inputZ;
  }
  _originFootPosition.x = inputX;
  _originFootPosition.y = inputY;
  _originFootPosition.z = inputZ;

  _IMUData.x = 0;
  _IMUData.y = 0;
  _IMUData.z = 0;
  _filteredIMUData.x = 0;
  _filteredIMUData.y = 0;
  _filteredIMUData.z = 0;
  _haveIMUFeedback = false;

};


/*!
 *    @brief Loads a gait that can be used with walk()
 *    @param gaitParameters An array of gait parameters generated by gaitCreator.py
 *    @param gaitSchedule   Gait step schedule generated by gaitCreator.py
*/
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
  for (int16_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    legStepPlanner[leg].setGaitParameters(_gait.amplitude, _gait.drawBackReduction);
  }
}

/*!
 *    @brief Sets robot into walk mode and manages gait schedule
 *    @param controlCoordinateX X coordinate of a joystick
 *    @param controlCoordinateY Y coordinate of a joystick
*/
void Quadruped::walk(int16_t controlCoordinateX, int16_t controlCoordinateY) {

  if (_mode == STANDING) {
    _mode = WALKING;
    _firstStep = true;
    _previousStepUpdate = millis();
    _justUpdatedWalk = false;
    _currentGaitScheduleIndex = 0;
  }

  if (_mode == WALKING) {
    if (((int16_t)(millis() - _previousStepUpdate) == (_gait.stepDuration + _gait.pauseDuration)) && _justUpdatedWalk == false) {

      int16_t stepDistance = 0;
      int16_t drawBackDistance = 0;

      // Weird stuff here; fix later. 
      if (_firstStep && _currentGaitScheduleIndex != _gait.stepCount - 1) {
        if (_gait.stepCount == 2) {
          stepDistance = _gait.stepDistance / 2;
          drawBackDistance = _gait.stepDistance / 2;
        }
        else if (_gait.stepCount == 4) {
          stepDistance = (2*(_currentGaitScheduleIndex) + 1) * _gait.stepDistance / 6;
          drawBackDistance = _gait.stepDistance / 6;
        }
      }
      else {
        if (_gait.stepCount == 2) {
          drawBackDistance = _gait.stepDistance;
        }
        else if (_gait.stepCount == 4) {
          drawBackDistance = _gait.stepDistance / 3;    // Notice that this causes innaccuracies; step distances that aren't multiples of 3 will have a step error. This has been 
                                                        // fixed by making all step distances in a 
        }
        stepDistance = _gait.stepDistance;
      }

      for (int16_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
        switch (_gaitSchedule[_currentGaitScheduleIndex][leg]) {
          case TAKE_STEP:
            legStepPlanner[leg].requestStep(controlCoordinateX, controlCoordinateY, _gait.stepDuration, stepDistance);
            break;
          case DRAW_BACK:
            legStepPlanner[leg].requestDrawBack(controlCoordinateX, controlCoordinateY, _gait.stepDuration, drawBackDistance);
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
      _previousStepUpdate = millis();
    }

    if ((millis() - _previousStepUpdate) % _gait.stepDuration != 0) _justUpdatedWalk = false;

  }

  for (int16_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    if (legStepPlanner[leg].update()) {
      _footPositions[leg].x = legStepPlanner[leg].dynamicFootPosition.x;
      _footPositions[leg].y = legStepPlanner[leg].dynamicFootPosition.y;
      _footPositions[leg].z = legStepPlanner[leg].dynamicFootPosition.z;
    }
  }

};


void Quadruped::giveIMUFeedback(float accelX, float accelY, float accelZ) {
  _IMUData.x = accelX;
  _IMUData.y = accelY;
  _IMUData.z = accelZ;
  _haveIMUFeedback = true;
}


/*!
 *    @brief Calculates the roll and pitch angles (YXZ rotation sequence) of the robot based on data provided to giveIMUFeedback(). 
      Refer to this paper from nxp: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf 
 *    @param roll A pointer to roll angle variable
 *    @param pitch A pointer to pitch angle variable
*/
void Quadruped::getRollPitch(float *roll, float *pitch) {
  if (!_haveIMUFeedback) return;
  
  float _roll, _pitch;

  _filteredIMUData.x = _IMUData.x * LPF_SMOOTHING_FACTOR + (_filteredIMUData.x * (1.0 - LPF_SMOOTHING_FACTOR));
  _filteredIMUData.y = _IMUData.y * LPF_SMOOTHING_FACTOR + (_filteredIMUData.y * (1.0 - LPF_SMOOTHING_FACTOR));
  _filteredIMUData.z = _IMUData.z * LPF_SMOOTHING_FACTOR + (_filteredIMUData.z * (1.0 - LPF_SMOOTHING_FACTOR));

  _roll = (atan2(-_filteredIMUData.y, _filteredIMUData.z) * 180)/PI;

#ifdef IMU_FLIPPED
  if (_roll < 0) _roll += 180;
  else if (_roll > 0) _roll -= 180;
#endif

  _pitch = (atan2(_filteredIMUData.x, sqrt(pow(_filteredIMUData.y,2) + pow(_filteredIMUData.z, 2)))*180)/PI;

  *roll = _roll;
  *pitch = _pitch;
}


void Quadruped::computeStaticMovement(int16_t offsetX, int16_t offsetY, int16_t offsetZ, int16_t rollAngle, int16_t pitchAngle, int16_t yawAngle) {

  int16_t offsetXL, offsetYL, rollAngleL, pitchAngleL, yawAngleL;

  // Constraints
  if (abs(offsetX) > (BODY_LENGTH * (PERCENT_LENGTH_TRANSLATION / 100))) {
    offsetXL = (int16_t)((float)BODY_LENGTH * ((float)PERCENT_LENGTH_TRANSLATION / 100));
    if (offsetX < 0)  offsetXL *= -1;
    offsetX = offsetXL;
  } 

  if (abs(offsetY) > (BODY_WIDTH * (PERCENT_WIDTH_TRANSLATION/100))) {
    offsetYL = (int16_t)((float)BODY_WIDTH * ((float)PERCENT_WIDTH_TRANSLATION / 100));
    if (offsetY < 0)  offsetYL *= -1;
    offsetY = offsetYL;
  } 

  if (abs(rollAngle) > ROLL_MAXIMUM_ANGLE) {
    rollAngleL = ROLL_MAXIMUM_ANGLE;
    if (rollAngle < 0)  rollAngleL *= -1;
    rollAngle = rollAngleL;
  }

  if (abs(pitchAngle) > PITCH_MAXIMUM_ANGLE) {
    pitchAngleL = PITCH_MAXIMUM_ANGLE;
    if (pitchAngle < 0)  pitchAngleL *= -1;
    pitchAngle = pitchAngleL;
  }

  if (abs(yawAngle) > YAW_MAXIMUM_ANGLE) {
    yawAngleL = YAW_MAXIMUM_ANGLE;
    if (yawAngle < 0)  yawAngleL *= -1;
    yawAngle = yawAngleL;
  }

  rollAngleL = rollAngle; 
  pitchAngleL = pitchAngle;
  yawAngleL = yawAngle;

  for (int8_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {

    _footPositions[leg].x = _originFootPosition.x;
    _footPositions[leg].y = _originFootPosition.y;
    _footPositions[leg].z = _originFootPosition.z;

    int16_t footYOffset, twistXOffset, twistYOffset, twistZOffset;

    // Apply X-axis offset while assuming a pitch
    _footPositions[leg].z -= offsetX * sin(pitchAngleL * (PI / 180));
    _footPositions[leg].x -= offsetX * cos(pitchAngleL * (PI / 180));

     // Apply Y-axis offset while assuming a roll
    _footPositions[leg].z -= offsetY * sin(rollAngleL * (PI / 180));
    footYOffset = offsetY * cos(rollAngleL * (PI / 180));   
    if (leg == 0 || leg == 3) _footPositions[leg].y -= footYOffset;
    if (leg == 1 || leg == 2) _footPositions[leg].y += footYOffset;

    // (Used for both calculations)
    float shoulderToGround = 0;

    // Pitch
    if (leg == 0 || leg == 1) 
      shoulderToGround = (float)_originFootPosition.z - (sin((float)pitchAngleL * ((float)PI / 180)) * ((float)BODY_LENGTH / 2));
    else if (leg == 2 || leg == 3) 
      shoulderToGround = (float)_originFootPosition.z + (sin((float)pitchAngleL * ((float)PI / 180)) * ((float)BODY_LENGTH / 2));

    _footPositions[leg].z += (cos(pitchAngleL * (PI / 180)) * shoulderToGround) - (_originFootPosition.z + offsetZ);
    _footPositions[leg].x += (sin(pitchAngleL * (PI / 180)) * shoulderToGround); 

    twistZOffset = sin(pitchAngleL * (PI / 180)) * ((BODY_LENGTH / 2) - cos(pitchAngleL * (PI / 180)) * (BODY_LENGTH / 2));
    twistXOffset = cos(pitchAngleL * (PI / 180)) * ((BODY_LENGTH / 2) - cos(pitchAngleL * (PI / 180)) * (BODY_LENGTH / 2));

    if (shoulderToGround > _originFootPosition.z) {
      _footPositions[leg].z += twistZOffset;
      _footPositions[leg].x -= twistXOffset;
    }
    else if (shoulderToGround < _originFootPosition.z) {
      _footPositions[leg].z -= twistZOffset;
      _footPositions[leg].x += twistXOffset;
    }

    // Roll
    if (leg == 0 || leg == 3) 
      shoulderToGround = (float)_originFootPosition.z - (sin((float)rollAngleL * ((float)PI / 180)) * ((float)BODY_WIDTH / 2));
    else if (leg == 1 || leg == 2) 
      shoulderToGround = (float)_originFootPosition.z + (sin((float)rollAngleL * ((float)PI / 180)) * ((float)BODY_WIDTH / 2));

    _footPositions[leg].z += (cos(rollAngleL * (PI / 180)) * shoulderToGround) - (_originFootPosition.z + offsetZ);

    footYOffset = (sin(rollAngleL * (PI / 180)) * shoulderToGround);
    if (leg == 0 || leg == 3) _footPositions[leg].y += footYOffset;
    if (leg == 1 || leg == 2) _footPositions[leg].y -= footYOffset;

    twistZOffset = sin(rollAngleL * (PI / 180)) * ((BODY_WIDTH / 2) - cos(rollAngleL * (PI / 180)) * (BODY_WIDTH / 2));
    twistYOffset = cos(rollAngleL * (PI / 180)) * ((BODY_WIDTH / 2) - cos(rollAngleL * (PI / 180)) * (BODY_WIDTH / 2));

    if (shoulderToGround > _originFootPosition.z) {
      _footPositions[leg].z += twistZOffset;
      _footPositions[leg].y -= twistYOffset;
    }
    else if (shoulderToGround < _originFootPosition.z) {
      _footPositions[leg].z -= twistZOffset;
      _footPositions[leg].y += twistYOffset;
    }

  }
}



/*!
 *    @brief Does the full computation for the foot position while considering all user inputs.
 *    @param inputX When in Standing mode, this is a translation in the X direction. When in Walking mode, this represents a X-axis controller input. 
 *    @param inputY When in Standing mode, this is a translation in the Y direction. When in Walking mode, this represents a Y-axis controller input. 
*/
void Quadruped::compute(RobotMode desiredMode, int16_t inputX, int16_t inputY, int16_t inputZ, int16_t rollAngle, int16_t pitchAngle, int16_t yawAngle) {
  if (desiredMode == STANDING) {
    computeStaticMovement(inputX, inputY, inputZ, rollAngle, pitchAngle, yawAngle);
  }

  else if (_mode == WALKING) {
    walk(inputX, inputY);
  }

  for (int16_t leg = 0; leg < ROBOT_LEG_COUNT; leg++) {
    legKinematics[leg].setFootEndpoint(_footPositions[leg].x, _footPositions[leg].y, _footPositions[leg].z);
  }

}


/*!
 *    @brief Returns a LegID enum depending on the index of the leg
 *    @returns LegID enum
*/
LegID Quadruped::_enumFromIndex(int8_t index) {
  if (index == 0) return LEG_1;
  else if (index == 1) return LEG_2;
  else if (index == 2) return LEG_3;
  else if (index == 3) return LEG_4;
  else return LEG_1;
};

