#include "StepPlanner.h"

StepPlanner::StepPlanner(void) {};

/*!
 *    @brief Initializes the gaits and sets up the leg modes
 *    @param legID The leg this stepPlanner object is used for
 *    @param offsetX The offset x direction from x = 0... you can have the walking occur further back or further forwards.
 *    @param offsetY The offset y direction from y = 0... you can have the walking occur further right or further left.
 *    @param robotHeight the default hieght of the robot. Every step will end in the leg being this height
*/
void StepPlanner::init(LegID legID, int16_t originXOffset, int16_t originYOffset, int16_t robotHeight) {

  _legID = legID;

  _mode = STANDING;
  _robotHeight = robotHeight;

  _originXOffset = originXOffset;
  _originYOffset = originYOffset;

  _stepOffsetX = 0;
  _stepOffsetY = 0;

  returnToOrigin();
}

/*!
 *    @brief Sets gait parameters like the amplitude and drawBackReduction for leg trajectory calculations
*/
void StepPlanner::setGaitParameters(int16_t amplitude, int16_t drawBackReduction) {
  _gaitAmplitude = amplitude;
  _gaitDrawBackReduction = drawBackReduction;
};

/*!
 *    @brief Updates the position of the foot as the step progresses.
 *    @returns True when the foot position was updated, false if it wasn't.
*/
bool StepPlanner::update() {

  if ((millis() - _previousUpdateTime) % GAIT_UPDATE_FREQUENCY == 0) {

    dynamicFootPosition.x = footPosX.update() + _originXOffset;
    dynamicFootPosition.y = footPosY.update() + _originYOffset;
    dynamicFootPosition.z = getStepHeight(footDrop.update());

    _previousUpdateTime = (millis() - 1);
    return true;
  }
  return false;
};

/*!
 *    @brief Requests a step in a direction, with a duration, and a distance.
 *    @param controlCoordinateX X coordinate of a joystick
 *    @param controlCoordinateY Y coordinate of a joystick
 *    @param stepDuration Duration of the step
 *    @param stepDistance The distance that the step should cover
*/
void StepPlanner::requestStep(int16_t controlCoordinateX, int16_t controlCoordinateY, int16_t stepDuration, int16_t stepDistance) {

  _mode = WALKING;
  _walkingStage = ACTIVE_WALKING_ARC;

  _gaitStepLength = stepDistance;

  returnToOrigin(true);

  setStepEndpoint(controlCoordinateX, controlCoordinateY, stepDistance);

  footPosX.go(_stepEndpoint.x, stepDuration, LINEAR, ONCEFORWARD);
  footPosY.go(_stepEndpoint.y, stepDuration, LINEAR, ONCEFORWARD);
  footDrop.go(stepDistance, stepDuration, LINEAR, ONCEFORWARD);
}

/*!
 *    @brief Requests a draw back in a direction, with a duration, and a distance.
 *    @param controlCoordinateX X coordinate of a joystick
 *    @param controlCoordinateY Y coordinate of a joystick
 *    @param stepDuration Duration of the draw back
 *    @param stepDistance The distance that the draw back should cover
*/
void StepPlanner::requestDrawBack(int16_t controlCoordinateX, int16_t controlCoordinateY, int16_t stepDuration, int16_t stepDistance) {

  _mode = WALKING;
  _walkingStage = ACTIVE_WALKING_DRAW_BACK;

  _gaitDrawBackLength = stepDistance;

  returnToOrigin(true);

  setStepEndpoint(controlCoordinateX, controlCoordinateY, stepDistance);

  footPosX.go(_stepEndpoint.x, stepDuration, LINEAR, ONCEFORWARD);
  footPosY.go(_stepEndpoint.y, stepDuration, LINEAR, ONCEFORWARD);
  footDrop.go(-1 * (stepDistance), stepDuration, LINEAR, ONCEFORWARD);
}

/*!
 *    @brief Updates the direction of the step or drawback.
 *    @param newControlCoordinateX X coordinate of a joystick
 *    @param newControlCoordinateY Y coordinate of a joystick
*/
void StepPlanner::updateEndpoint(int16_t newControlCoordinateX, int16_t newControlCoordinateY) {

  setStepEndpoint(newControlCoordinateX, newControlCoordinateY, _gaitDrawBackLength);

  // footPosY would work also
  long timeLeft = (footPosX.getDuration() - ((footPosX.getCompletion() / 100) * footPosX.getDuration()));

  footPosX.go(_stepEndpoint.x, timeLeft, LINEAR, ONCEFORWARD);
  footPosY.go(_stepEndpoint.y, timeLeft, LINEAR, ONCEFORWARD);
}


bool StepPlanner::applyStepOffset(int16_t offsetX, int16_t offsetY, int16_t shiftDuration) {

  long  timeLeft = (footPosX.getDuration() - ((footPosX.getCompletion() / 100) * footPosX.getDuration()));

  if (timeLeft == 0) {
    if (shiftDuration > 0) timeLeft = shiftDuration;
    else return false;
  }

  _stepEndpoint.x = _stepEndpoint.x - _stepOffsetX + offsetX;
  _stepEndpoint.y = _stepEndpoint.y - _stepOffsetY + offsetY;

  // _stepEndpoint.x += offsetX;
  // _stepEndpoint.y += offsetY;

  footPosX.go(_stepEndpoint.x, timeLeft, LINEAR, ONCEFORWARD);
  footPosY.go(_stepEndpoint.y, timeLeft, LINEAR, ONCEFORWARD);

  _stepOffsetX = offsetX;
  _stepOffsetY = offsetY;

  return true;

}

/*!
 *    @brief Calculates the endpoint of the step in a direction.
 *    @param controlCoordinateX X coordinate of a joystick
 *    @param controlCoordinateY Y coordinate of a joystick
 *    @param stepDistance The distance that the draw back should cover
*/
void StepPlanner::setStepEndpoint(int16_t controlCoordinateX, int16_t controlCoordinateY, int16_t stepDistance) {

  float stepEndpointX = 0.0;
  float stepEndpointY = 0.0;
  
  float movementGradient = 0;

  if (controlCoordinateX == 0) {
    stepEndpointX = 0;
    stepEndpointY = stepDistance;
  }
  else {
    movementGradient = (float)(controlCoordinateY / controlCoordinateX);

    stepEndpointX = (stepDistance / sqrt(1 + pow(movementGradient, 2)));
    stepEndpointY = ((stepDistance * abs(movementGradient)) / sqrt(1 + pow(movementGradient, 2)));
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
  if ((_walkingStage == ACTIVE_WALKING_DRAW_BACK) || (_walkingStage == FIRST_STEP_DRAW_BACK)) {
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
  _stepEndpoint.x = stepEndpointY + footPosX.update();
  _stepEndpoint.y = stepEndpointX + footPosY.update();

}




/*!
 *    @brief Performs the calculation of the arc itself ONLY... doesn't know where the foot actually is
 *    @param footXYDropL The x drop position of the foot.
 *    @param legMode The mode/phase of walking the leg is in.
 *    @returns The hight that should be written to the legs i.e. foot z distance (foot-should) - curve hight at the given footXYDropL
*/
int16_t StepPlanner::getStepHeight(float footXYDropL) {

  int16_t stepHeight = 0;

  // float periodHalf = _gait.periodHalf;
  // float amplitude = _gait.amplitude;

  if (_mode == STANDING)
    return _robotHeight;

  switch (_walkingStage) {
    case FIRST_STEP_ARC:           stepHeight = _robotHeight - lrint( (_gaitAmplitude/2) * sin(PI * (footXYDropL - (_gaitStepLength/4))/(_gaitStepLength/2) ) ); break;
    case FIRST_STEP_DRAW_BACK:     stepHeight = _robotHeight - 0; break;
    case ACTIVE_WALKING_ARC:       stepHeight = _robotHeight - lrint( _gaitAmplitude * sin( (PI * (footXYDropL)/_gaitStepLength) ) ); break;
    case ACTIVE_WALKING_DRAW_BACK: 
      if (_gaitDrawBackReduction == 0)  stepHeight = _robotHeight - 0;
      else stepHeight = _robotHeight - lrint( (_gaitAmplitude/_gaitDrawBackReduction) * sin(PI * (footXYDropL)/_gaitDrawBackLength ) );
      break;
  }

  return (int16_t)lrint(stepHeight);

};


/*!
 *    @brief Sets the interpolation objects back to 0 for resets.
 *    @param footDropOnly Whether only the footDrop should be reset. If true, it is the only one reset. It false, all objects are reset. 
*/
void StepPlanner::returnToOrigin(bool footDropOnly) {
  if (footDropOnly) {
    footDrop.go(0);
    return;
  }
  footPosX.go(0);
  footPosY.go(0);
}


/*!
 *    @brief Returns whether the interpolation objects have reached their endpoints (whether the foot is done moving for a command)
 *    @returns true if the previous action was completed, false if it wasn't.
*/
bool StepPlanner::finishedAction() {
  if (footPosX.isFinished() && footPosY.isFinished() && footDrop.isFinished())
    return true;
  else
    return false;
}

/*!
 *    @brief Allows you to set a new height for the robot
 *    @param newHeight  The new height of the robot
*/
void StepPlanner::setNewHeight(int16_t newHeight) {
  _robotHeight = newHeight;
}