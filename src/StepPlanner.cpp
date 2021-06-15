#include "StepPlanner.h"

StepPlanner::StepPlanner(LegID legID) {
  _legID = legID;
};

void StepPlanner::init() {

  _legMode = STANDING;
  _gaitType = DEFAULT_GAIT;

  _gaits[TROT].amplitude = 50;
  _gaits[TROT].periodHalf = 140;

  dynamicFootPosition.x = 0;
  dynamicFootPosition.y = 0;
  dynamicFootPosition.z = 177;

  _footXYDrop = 0;

  _robotHeight = 177;

  footPosX.go(0);
  footPosY.go(0);
}

void StepPlanner::setGait(GaitType gaitType) {
  if (_legMode == STANDING)
    _gaitType = gaitType;
}

void StepPlanner::update(ROBOT_MODE robotMode) {
  

  // Make sure that the leg needs to stand AND the robot should stand. 
  if ((_legMode == STANDING) && (robotMode == WALKING)) {
    _previousUpdateTime = (millis() - 1);

    /// Determine which foot goes first
    if (_legID == LEG_1 || _legID == LEG_3) {
      _legMode = FIRST_STEP_ARC;
      _footXYDrop  = 0;
    }
    else if (_legID == LEG_2 || _legID == LEG_4) {
      _legMode = FIRST_STEP_DRAW_BACK;
      _footXYDrop  = 0;
    }
    return;
  }
  // else if ((_legMode == STANDING) && (robotMode == STATIC_STANDING))


  float periodHalf = _gaits[_gaitType].periodHalf;

  if ((millis() - _previousUpdateTime) % TIME_TO_UPDATE == 0) {

    dynamicFootPosition.x = footPosX.update();
    dynamicFootPosition.y = footPosY.update();
    dynamicFootPosition.z = getStepHeight(_footXYDrop, _legMode);

    switch (_legMode) {
      case FIRST_STEP_ARC:            
        _footXYDrop += GAIT_POSITION_INCREMENT; 
        if (_footXYDrop == (periodHalf/2)) {
          _legMode = ACTIVE_WALKING_DRAW_BACK;
        }
        break;
      case FIRST_STEP_DRAW_BACK:      
        _footXYDrop -= GAIT_POSITION_INCREMENT; 
        if (_footXYDrop == -1*(periodHalf/2)) {
          _legMode = ACTIVE_WALKING_ARC;
        }
        break;
      case ACTIVE_WALKING_ARC:
        _footXYDrop += GAIT_POSITION_INCREMENT;
        if (_footXYDrop == (periodHalf/2)) {
          _legMode = ACTIVE_WALKING_DRAW_BACK;
        }
        break;
      case ACTIVE_WALKING_DRAW_BACK:
        _footXYDrop -= GAIT_POSITION_INCREMENT;
        if (_footXYDrop == -1*(periodHalf/2)) {
          _legMode = ACTIVE_WALKING_ARC;
        }
        break;
      case STANDING:
        break;
    }
  }

      _previousUpdateTime = (millis() - 1);

};

/*!
 *    @brief Performs the calculation of the arc itself ONLY... doesn't know where the foot actually is
*/
int16_t StepPlanner::getStepHeight(int16_t footXYDropL, LegMode legMode) {

  int16_t stepHeight = 0;

  float periodHalf = _gaits[_gaitType].periodHalf;
  float amplitude = _gaits[_gaitType].amplitude;

  switch (legMode) {
    case FIRST_STEP_ARC:           stepHeight = _robotHeight - lrint( (amplitude/2) * cos(PI * (footXYDropL - (periodHalf/4))/(periodHalf/2) ) ); break;
    case FIRST_STEP_DRAW_BACK:     stepHeight = 0; break;
    case ACTIVE_WALKING_ARC:       stepHeight = _robotHeight - lrint( amplitude * cos( (PI * (footXYDropL)/periodHalf) ) ); break;
    case ACTIVE_WALKING_DRAW_BACK: stepHeight = 0; break;
    case STANDING:                 stepHeight = 0;
  }

  return stepHeight;

};


void StepPlanner::setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, ROBOT_MODE robotMode) {
  // Find the coordinate point to walk to for each leg while considering the direcion to walk in.

  float periodHalf = _gaits[_gaitType].periodHalf;

  // Stopped walking
  if (robotMode == STATIC_STANDING) {
    _stepEndpoint.x = 0;
    _stepEndpoint.y = 0;
    _legMode = STANDING;
  }
  // make sure that math works (vertical lines are undefined)
  else if (controlCoordinateX == 0 && controlCoordinateY != 0) {
    _stepEndpoint.x = 0;
    _stepEndpoint.y = periodHalf/2;
  }
  else {
    float movementGradient = (controlCoordinateY / controlCoordinateX);

    _stepEndpoint.x = ((periodHalf/2) / sqrt(1 + pow(movementGradient, 2)));
    _stepEndpoint.y = (((periodHalf/2) * movementGradient) / sqrt(1 + pow(movementGradient, 2)));
  }

  if (controlCoordinateX < 0)
    _stepEndpoint.x *= -1;
  if (controlCoordinateY < 0)
    _stepEndpoint.y *= -1;

  if ((_legMode == ACTIVE_WALKING_DRAW_BACK) || (_legMode == FIRST_STEP_DRAW_BACK)) {
    _stepEndpoint.x *= -1;
    _stepEndpoint.y *= -1;
  }
    

  long completionTime = (long)((TIME_TO_UPDATE - 1) * (periodHalf/(2 * GAIT_POSITION_INCREMENT)));

  footPosX.go(_stepEndpoint.x, completionTime, LINEAR, FORTHANDBACK);
  footPosY.go(_stepEndpoint.y, completionTime, LINEAR, FORTHANDBACK);
};


/*!
 *    @brief Used to figure out if it's time to update the step endpoint
 *    @returns True if it's time to update the endpoint, false if it's not.
*/
bool StepPlanner::footAtOrigin() {
  if (_footXYDrop != 0) 
    return false;
  return true;
}