#include "StepPlanner.h"

StepPlanner::StepPlanner(void) {};

/*!
 *    @brief Initializes the gaits and sets up the leg modes
 *    @param legID The leg this stepPlanner object is used for
 *    @param offsetX The offset x direction from x = 0... you can have the walking occur further back or further forwards.
 *    @param offsetY The offset y direction from y = 0... you can have the walking occur further right or further left.
 *    @param robotHeight the default hieght of the robot. Every step will end in the leg being this height
*/
void StepPlanner::init(LegID legID, int16_t offsetX, int16_t offsetY, int16_t robotHeight) {

  _legID = legID;

  _legMode = STANDING;
  _gaitType = DEFAULT_GAIT;
  _robotHeight = robotHeight;

  _offsetX = offsetX;
  _offsetY = offsetY;

  _gaits[TROT].amplitude = 60;
  _gaits[TROT].periodHalf = 40;

  reset();
}

/*!
 *    @brief Sets the walking giat. Should only be called when the robot is standing
*/
void StepPlanner::setGait(GaitType gaitType) {
  if (_legMode == STANDING)
    _gaitType = gaitType;
}

/*!
 *    @brief Updates the position of the foot as the step progresses.
 *    @param robotMode The mode the robot is in i.e. walking, static_standing.
             used to determine when the first step and the last step are being made
 *    @returns True when the foot position was updated, false if it wasn't.
*/
bool StepPlanner::update(ROBOT_MODE robotMode) {

  float periodHalf = _gaits[_gaitType].periodHalf;

  if ((millis() - _previousUpdateTime) % TIME_TO_UPDATE == 0) {

    if (abs(_footXYDrop) == (periodHalf / 2)) {
      footPosX.pause();
      footPosY.pause();
      delay(INTER_STEP_PAUSE);
      footPosX.resume();
      footPosY.resume();
    }


    // For legs 2 and 3, the negative and positive parts of the x axis are flipped

    // dynamicFootPosition.x = footPosX.update() + _offsetX;
    // dynamicFootPosition.y = footPosY.update() + _offsetY;
    dynamicFootPosition.x = _footXYDrop + _offsetX;
    dynamicFootPosition.y = 0;
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
    _previousUpdateTime = (millis() - 1);
    return true;
  }
  return false;
};

/*!
 *    @brief Performs the calculation of the arc itself ONLY... doesn't know where the foot actually is
 *    @param footXYDropL The x drop position of the foot.
 *    @param legMode The mode/phase of walking the leg is in.
 *    @returns The hight that should be written to the legs i.e. foot z distance (foot-should) - curve hight at the given footXYDropL
*/
int16_t StepPlanner::getStepHeight(int16_t footXYDropL, LegMode legMode) {

  int16_t stepHeight = 0;

  float periodHalf = _gaits[_gaitType].periodHalf;
  float amplitude = _gaits[_gaitType].amplitude;

  switch (legMode) {
    case FIRST_STEP_ARC:           stepHeight = _robotHeight - lrint( (amplitude/2) * cos(PI * (footXYDropL - (periodHalf/4))/(periodHalf/2) ) ); break;
    case FIRST_STEP_DRAW_BACK:     stepHeight = _robotHeight - 0; break;
    case ACTIVE_WALKING_ARC:       stepHeight = _robotHeight - lrint( amplitude * cos( (PI * (footXYDropL)/periodHalf) ) ); break;
    // case ACTIVE_WALKING_DRAW_BACK: stepHeight = _robotHeight - 0; break;
    case ACTIVE_WALKING_DRAW_BACK: stepHeight = _robotHeight + lrint( (amplitude/DRAW_BACK_AMPLITUDE_REDUCTION) * cos(PI * (footXYDropL)/periodHalf ) ); break;
    case STANDING:                 stepHeight = _robotHeight - 0;
  }

  return stepHeight;

};

/*!
 *    @brief Allows you to set the endpoint of the step depending on the direction you command.
            This also handles whether the foot is taking its first or last step. 
 *    @param controllCoordinateX X direction of the controller coordinate 
 *    @param controllCoordinateY Y direction of the controller coordinate 
 *    @returns True if it's time to update the endpoint, false if it's not.
*/
void StepPlanner::setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, ROBOT_MODE robotMode, int16_t yawOffset) {

  // If this is the first step, it must be determined whether to arc or draw back and the time of last update must be set.
  if (_setFirstStep(robotMode)) _previousUpdateTime = (millis() - 1);

  float stepEndpointX = 0.0;
  float stepEndpointY = 0.0;

  float periodHalf = _gaits[_gaitType].periodHalf;
  float movementGradient = 0;

  // Check if stopped walking
  if (controlCoordinateX == 0 && controlCoordinateY == 0) {
    _stepEndpoint.x = 0.0;
    _stepEndpoint.y = 0.0;
    footPosX.go(_stepEndpoint.x);
    footPosY.go(_stepEndpoint.y);

#if !defined(STANDING_TROT)
    _legMode = STANDING;
#endif

    return;
  }

  // make sure that math works (vertical lines are undefined)
  else if (controlCoordinateX == 0) {
    stepEndpointX = 0;
    stepEndpointY = periodHalf/2;
  }
  else {
    movementGradient = (float)(controlCoordinateY / controlCoordinateX);

    stepEndpointX = ((periodHalf/2) / sqrt(1 + pow(movementGradient, 2)));
    stepEndpointY = (((periodHalf/2) * abs(movementGradient)) / sqrt(1 + pow(movementGradient, 2)));
  }

  // verify that the direction is correct. The quadrant of movement must 
  // be accounted for in order to match the joystick. 
  if (controlCoordinateX < 0) stepEndpointX *= -1;
  if (controlCoordinateY < 0) stepEndpointY *= -1;


  // apply the yaw calculation
  // if ((_legID == LEG_2) || (_legID == LEG_4))
  //   stepEndpointX -= yawOffset;
  // else if ((_legID == LEG_1) || (_legID == LEG_3))
  //   stepEndpointX += yawOffset;

  // drawback gaits are pushing to a position opposite from the arc position
  if ((_legMode == ACTIVE_WALKING_DRAW_BACK) || (_legMode == FIRST_STEP_DRAW_BACK)) {
    stepEndpointX *= -1;
    stepEndpointY *= -1;
  }

  // The kinematics engine thinks that specifying a negative y distance
  // means that the foot should move into the robot. If legs 2 or 3 need to 
  // move positively right (positive stepEndpoint), they are actually moving 
  // in the negative direction for kinematics. 
  if ((_legID == LEG_2) || (_legID == LEG_3))
    stepEndpointX *= -1;

  // flip the result (kinematics thinks that x is moving forwards/backwards while looking down the robot)
  // stepPlanner thinks that y is moving forwards/backwards
  _stepEndpoint.x = stepEndpointY;
  _stepEndpoint.y = stepEndpointX;

  long completionTime = (long)((TIME_TO_UPDATE - 1) * (periodHalf/(2 * GAIT_POSITION_INCREMENT)));

  // Ideally, the interpolation objects will effectively track the position of footXY drop.
  // When it reaches its maximum point, so does footXY drop (likewise when they are 0).

  footPosX.go(_stepEndpoint.x, completionTime, LINEAR, FORTHANDBACK);
  footPosY.go(_stepEndpoint.y, completionTime, LINEAR, FORTHANDBACK);
};


// footAtOrigin updates whenever footXYDrop == 0 since this is the only time that all feet 
// can be updated with the same endpoint and have the same distance to travel in order to get
// to the endpoint. While the step could be updated every cycle, this reduces responsivity
// (the current setup essentially allows for an update every half-cycle).
/*!
 *    @brief Used to figure out if it's time to update the step endpoint
 *    @returns True if it's time to update the endpoint, false if it's not.
*/
bool StepPlanner::footAtOrigin() {
  if ((_footXYDrop == 0) && (_wasAtOrigin == false)) {
    _wasAtOrigin = true;
    return true;
  }
  if ((_footXYDrop != 0))
    _wasAtOrigin = false;
  return false;
}

/*!
 *    @brief Resets all dynamic gait parameters
*/
void StepPlanner::reset() {
  dynamicFootPosition.x = 0;
  dynamicFootPosition.y = 0;
  dynamicFootPosition.z = _robotHeight;

  _wasAtOrigin = false;
  _legMode = STANDING;
  _footXYDrop = 0;

  footPosX.go(0);
  footPosY.go(0);
}

/*!
 *    @brief Sets the first step to FIRST_STEP_ARC or FIRST_STEP_DRAW_BACK
 *    @param robotMode the mode of the robot.
 *    @returns true if the first step was set (it was called on the first step),
 *             false if it wasn't.
*/
bool StepPlanner::_setFirstStep(ROBOT_MODE robotMode) {

  if ((_legMode == STANDING) && (robotMode == WALKING)) {

#if defined(RIGHT_FOOTED)

    if (_legID == LEG_1 || _legID == LEG_3) {
      _legMode = FIRST_STEP_ARC;
    }
    else if (_legID == LEG_2 || _legID == LEG_4) {
      _legMode = FIRST_STEP_DRAW_BACK;
    }

#elif defined(LEFT_FOOTED)

    if (_legID == LEG_1 || _legID == LEG_3) {
      _legMode = FIRST_STEP_DRAW_BACK;
    }
    else if (_legID == LEG_2 || _legID == LEG_4) {
      _legMode = FIRST_STEP_ARC;
    }
#else
#error You must define whether your robot is RIGHT_FOOTED or LEFT_FOOTED in quadruped-config.h
#endif

    return true;
  }

  return false;
}