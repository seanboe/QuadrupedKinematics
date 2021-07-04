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

// void Quadruped::update(int16_t controlCoordinateX, int16_t controlCoordinateY) {
//   legStepPlanner[0].calculateStep(controlCoordinateX, controlCoordinateY);
// }



void Quadruped::walk(int16_t controlCoordinateX, int16_t controlCoordinateY) {

  if (legStepPlanner[0].update()) {
    int16_t inputX = legStepPlanner[0].dynamicFootPosition.x;
    int16_t inputY = legStepPlanner[0].dynamicFootPosition.y;
    int16_t inputZ = legStepPlanner[0].dynamicFootPosition.z;

    legKinematics[0].setFootEndpoint(inputX, inputY, inputZ);
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




};




int16_t Quadruped::computeYaw(int16_t yawAngle) {
  // Map the controller input to a yaw angle. It is assumed that 0 degrees is when the body of the robot is straight
  // forwards, with degrees increasing as you approach the rear of the robot. 
  if (abs(yawAngle) > YAW_MAXIMUM_ANGLE) {
    if (yawAngle > 0) yawAngle = YAW_MAXIMUM_ANGLE;
    if (yawAngle < 0) yawAngle = -1 * YAW_MAXIMUM_ANGLE;
  }

  // Serial.println(lrint((BODY_LENGTH / 2) * atan((yawAngle * PI)/180)));

  return lrint((BODY_LENGTH / 2) * atan((yawAngle * PI)/180));

};


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