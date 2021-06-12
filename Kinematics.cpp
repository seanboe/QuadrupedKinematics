#include "Kinematics.h"

#include <Arduino.h>


/*!
 *    @param  legID Leg number. Numbering follows the quadrants of a unit circle.
 */
Kinematics::Kinematics(LegID legID) {
  _legID = legID;
};


// *****************Private Functions*****************

/*!
 *    @brief  Converts degrees to microseconds for the motors. 
 *    @param  inputDegrees Degrees to be converted to microseconds
 *    @param  calibOffset Each motor's calibration offset
 *    @return The Microseconds converted from inputDegrees
 */
uint16_t Kinematics::_degreesToMicros(uint8_t inputDegrees, uint8_t calibOffset) {
  int microsecondsInput = ((DEGREES_TO_MICROS * inputDegrees) + 500 + calibOffset);    // 500 is a "magic number" of micros for the motors; before that they do nothing
  return microsecondsInput;
};


/*!
 *    @brief  Returns the index of a motor in the motor list given
 *            the leg it is in and the motor number of the leg 
 *            you want.
 *    @param  leg The leg that the motor is in. 
 *    @param  motor The motor in the leg you are trying to access
 *    @return The index of the motor
 */
uint16_t Kinematics::_indexOfMotor(LegID leg, MotorID motor) {
  return ((leg - 1) * MOTORS_PER_LEG + motor) - 1;
};


/*!
 *    @brief  Initializes the motor angles given the default position. Also does setup
 *    to integrate the external array of motors in the class.
 *    @param  inputX startup x-axis coordinate
 *    @param  inputY startup y-axis coordinate
 *    @param  inputZ startup z-axis coordinate
 *    @param  legMotors array of Motor variables for each motor
 */
void Kinematics::init(int16_t inputX, int16_t inputY, int16_t inputZ, Motor legMotors[]) {
  _motors = legMotors;

  // Solve for the initial foot position
  solveFootPosition(inputX, inputY, inputZ, &_motors[_indexOfMotor(_legID, M1)].angleDegrees, &_motors[_indexOfMotor(_legID, M2)].angleDegrees, &_motors[_indexOfMotor(_legID, M3)].angleDegrees);

  // Motor 1
  _motors[_indexOfMotor(_legID, M1)].angleMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M1)].angleDegrees, _motors[_indexOfMotor(_legID, M1)].calibOffset);
  _motors[_indexOfMotor(_legID, M1)].dynamicDegrees = _motors[_indexOfMotor(_legID, M1)].angleDegrees;
  _motors[_indexOfMotor(_legID, M1)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M1)].dynamicDegrees, _motors[_indexOfMotor(_legID, M1)].calibOffset);
  _motors[_indexOfMotor(_legID, M1)].previousDegrees = 360;    // 360 just needs to an angle that the motor can't be at... the motors can never achieve 360!

  // Motor 2
  _motors[_indexOfMotor(_legID, M2)].angleMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M2)].angleDegrees, _motors[_indexOfMotor(_legID, M2)].calibOffset);
  _motors[_indexOfMotor(_legID, M2)].dynamicDegrees = _motors[_indexOfMotor(_legID, M2)].angleDegrees;
  _motors[_indexOfMotor(_legID, M2)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M2)].dynamicDegrees, _motors[_indexOfMotor(_legID, M2)].calibOffset);
  _motors[_indexOfMotor(_legID, M2)].previousDegrees = 360;    // 360 just needs to an angle that the motor can't be at... the motors can never achieve 360!

  // Motor 3
  _motors[_indexOfMotor(_legID, M3)].angleMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M3)].angleDegrees, _motors[_indexOfMotor(_legID, M3)].calibOffset);
  _motors[_indexOfMotor(_legID, M3)].dynamicDegrees = _motors[_indexOfMotor(_legID, M3)].angleDegrees;
  _motors[_indexOfMotor(_legID, M3)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M3)].dynamicDegrees, _motors[_indexOfMotor(_legID, M3)].calibOffset);
  _motors[_indexOfMotor(_legID, M3)].previousDegrees = 360;    // 360 just needs to an angle that the motor can't be at... the motors can never achieve 360!

  dynamicX.go(inputX);
  dynamicY.go(inputY);
  dynamicZ.go(inputZ);
}


// *****************Public Functions*****************

/*!
 *    @brief  Sets the desired foot endpoint in Cartesian coordinates (mm)
 *    @param  inputX x-axis coordinate
 *    @param  inputY y-axis coordinate
 *    @param  inputZ z-axis coordinate
 */
void Kinematics::setFootEndpoint(int16_t inputX, int16_t inputY, int16_t inputZ) {

  solveFootPosition(inputX, inputY, inputZ, &_motors[_indexOfMotor(_legID, M1)].angleDegrees, &_motors[_indexOfMotor(_legID, M2)].angleDegrees, &_motors[_indexOfMotor(_legID, M3)].angleDegrees);

  uint16_t motor1AngleDelta = abs(_motors[_indexOfMotor(_legID, M1)].angleDegrees - _motors[_indexOfMotor(_legID, M1)].previousDegrees);
  uint16_t motor2AngleDelta = abs(_motors[_indexOfMotor(_legID, M2)].angleDegrees - _motors[_indexOfMotor(_legID, M2)].previousDegrees);
  uint16_t motor3AngleDelta = abs(_motors[_indexOfMotor(_legID, M3)].angleDegrees - _motors[_indexOfMotor(_legID, M3)].previousDegrees);
  uint16_t demandTime = lrint(MAX_SPEED_INVERSE * max(max(motor1AngleDelta, motor2AngleDelta), motor3AngleDelta));


    // determine whether motor angles have been updated i.e. new end angle, and update final positions accordingly
  if ((_motors[_indexOfMotor(_legID, M1)].previousDegrees != _motors[_indexOfMotor(_legID, M1)].angleDegrees)
   || (_motors[_indexOfMotor(_legID, M2)].previousDegrees != _motors[_indexOfMotor(_legID, M2)].angleDegrees) 
   || (_motors[_indexOfMotor(_legID, M3)].previousDegrees != _motors[_indexOfMotor(_legID, M3)].angleDegrees)) {
     
    _motors[_indexOfMotor(_legID, M1)].previousDegrees = _motors[_indexOfMotor(_legID, M1)].angleDegrees;
    _motors[_indexOfMotor(_legID, M2)].previousDegrees = _motors[_indexOfMotor(_legID, M2)].angleDegrees;
    _motors[_indexOfMotor(_legID, M3)].previousDegrees = _motors[_indexOfMotor(_legID, M3)].angleDegrees;

    dynamicX.go(inputX, demandTime, LINEAR, ONCEFORWARD);
    dynamicY.go(inputY, demandTime, LINEAR, ONCEFORWARD);
    dynamicZ.go(inputZ, demandTime, LINEAR, ONCEFORWARD);
  }
}

/*!
 *    @brief  Recalculates the foot position based on the interpolated axis
 *    @returns void
*/
void Kinematics::updateDynamicFootPosition() {

  solveFootPosition(dynamicX.update(), dynamicY.update(), dynamicZ.update(), &_motors[_indexOfMotor(_legID, M1)].dynamicDegrees, &_motors[_indexOfMotor(_legID, M2)].dynamicDegrees, &_motors[_indexOfMotor(_legID, M3)].dynamicDegrees);
  _motors[_indexOfMotor(_legID, M1)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M1)].dynamicDegrees, _motors[_indexOfMotor(_legID, M1)].calibOffset);
  _motors[_indexOfMotor(_legID, M2)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M2)].dynamicDegrees, _motors[_indexOfMotor(_legID, M2)].calibOffset);
  _motors[_indexOfMotor(_legID, M3)].dynamicMicros = _degreesToMicros(_motors[_indexOfMotor(_legID, M3)].dynamicDegrees, _motors[_indexOfMotor(_legID, M3)].calibOffset);
}

/*!
 *    @brief  Solves the angles needed to achieve a defined foot-to-shoulder length
 *    @param  demandFtShldr Desired foot-should length
 *    @param  demandAngle2  Angle to hold the output for motor 2
 *    @param  demandAngle3  Angle to hold the output for motor 2
 */
void Kinematics::solveFtShldrLength(float demandFtShldr, float *demandAngle2, float *demandAngle3) {
  
  float _demandFtShldrLength = demandFtShldr;
  if (_demandFtShldrLength > SHOULDER_FOOT_MAX) 
    _demandFtShldrLength = SHOULDER_FOOT_MAX;
  else if (_demandFtShldrLength < SHOULDER_FOOT_MIN)
    _demandFtShldrLength = SHOULDER_FOOT_MIN;

  // Use the Law of Cosines to solve for the angles of motor 3 and convert to degrees
  float _demandAngle3 = acos( ( pow(demandFtShldr, 2) - pow(LIMB_2, 2) - pow(LIMB_3, 2) ) / (-2 * LIMB_2 * LIMB_3) ); // demand angle for position 3 (operated by M3)
  _demandAngle3 = ((_demandAngle3 * 180) / PI);   //convert to degrees

  // Use demandAngle3 to calculate for demandAngle2 (angle for M2)
  float _demandAngle2 = ((180 - _demandAngle3) / 2 );

  *demandAngle2 += _demandAngle2;
  *demandAngle3 += _demandAngle3;
};


/*!
 *    @brief  Solves the angles needed to achieve a specified x-axis movement
 *    @param  inputX        The desired x-axis coordinate (mmm)
 *    @param  inputZ        The desired z-axis coordinate (mm)
 *    @param  demandAngle2  Angle to hold the output for motor 2
 *    @param  demandFtShldrLength   Outputted foot shoulder length
 */
void  Kinematics::solveXMove(int16_t inputX, int16_t inputZ, float *demandAngle2, float *demandFtShldrLength) {
  if (inputZ == 0)
    inputZ = 1;   // you can never divide by 0!

  *demandFtShldrLength = sqrt(pow((float)abs(inputZ), 2) + pow((float)abs(inputX), 2));

  *demandAngle2 = ((atan((float)abs(inputX)/(float)abs(inputZ))*180) / PI);

  if (inputX > 0)
    *demandAngle2 *= -1;            // change later: make it negative if inputX is in the negative direction and parse it later
};


/*!
 *    @brief  Solves the angles needed to achieve a specified y-axis movement
 *    @param  inputY        The desired y-axis coordinate (mmm)
 *    @param  inputZ        The desired z-axis coordinate (mm)
 *    @param  demandAngle2  Angle to hold the output for motor 1
 *    @param  yPlaneZOutput The desired z-axis coordinate on the z-y plane.
 *    When you shift the entire leg via a y-axis change, then the distance from
 *    the foot to the shoulder on the x-z plane changes than that on the y-z plane.
 *    This must be considered. Ideally, this output should be passed to all other 
 *    calculations as the z-axis coordinate. 
 */
void Kinematics::solveYMove(int16_t inputY, int16_t inputZ, float *demandAngle1, float *yPlaneZOutput) {
  float demandFtShldrLength = sqrt(pow((float)abs(inputZ), 2) + pow((float)abs(inputY), 2)); // foot-shoulder distance on y-z plane (L1 in diagram)
  *yPlaneZOutput = sqrt(pow((float)abs(demandFtShldrLength), 2) - pow((float)abs(LIMB_1), 2));

  // Here, theta is the angle closest to the axis of rotation in the triangle relating inputY and inputZ
  // Alpha is the angle closest to the axis of rotation in the triangle relating leg length output to LIMB_1 length
  float theta = (float)abs((((float)atan((float)inputY/(float)inputZ) * 180) / PI));
  float alpha = (float)(((float)acos((float)LIMB_1/demandFtShldrLength) * 180) / PI);
  if (inputY >= 0) {
    *demandAngle1 += (float)abs((float)90 - (theta + alpha));
  }
  else if (inputY < 0) {
    *demandAngle1 += (float)abs((float)90 - (alpha - theta));   // since both triangles (refer to drawings) have the same hypotenuse, alpha > theta for all inputY
  }


  // NEGATIVE signifies a direction: the foot is moving TOWARDS THE ROBOT
  // POSITIVE signifies AWAY FROM ROBOT
  if (inputY < LIMB_1)
    *demandAngle1 *= -1;

}


/*!
 *    @brief  Overall kinematics function that calculates all the angles for an x-y-z 
      coordinate foot position. 
 *    @param  inputX        The desired x-axis coordinate (mmm) 
 *    @param  inputY        The desired y-axis coordinate (mm)
 *    @param  inputZ        The desired z-axis coordinate (mm)
 *    @param  motor1AngleP  The motor 1 angle output
 *    @param  motor2AngleP  The motor 2 angle output
 *    @param  motor3AngleP  The motor 3 angle output
 */
void Kinematics::solveFootPosition(int16_t inputX, int16_t inputY, int16_t inputZ, int16_t *motor1AngleP, int16_t *motor2AngleP, int16_t *motor3AngleP) {
  float demandAngle1 = 0;
  float demandAngle2 = 0;
  float demandAngle3 = 0;

  float yPlaneZOutput = 0;  // this is the foot-shoulder distance on the y-z plane (L1 in diagram), and the distance the leg must stretch to achieve the desired y movement on the y-z plane. 
  float demandFtShldrLength = 0;  // this is the foot-should distance on the x-z plane and the final calculated length

  solveYMove(inputY, inputZ, &demandAngle1, &yPlaneZOutput);

  solveXMove(inputX, yPlaneZOutput, &demandAngle2, &demandFtShldrLength);

  solveFtShldrLength(demandFtShldrLength, &demandAngle2, &demandAngle3);

  // Round off demand angles
  demandAngle1 = lrint(demandAngle1);
  demandAngle2 = lrint(demandAngle2);
  demandAngle3 = lrint(demandAngle3);


  // Set live motor angles to the newly calculated ones

  //motor 1: 
  *motor1AngleP = demandAngle1; // In degrees!

  // motor 2:
  *motor2AngleP = demandAngle2; // In degrees!

  // motor 3:
  *motor3AngleP = demandAngle3; // In degrees!
};