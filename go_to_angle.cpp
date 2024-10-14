#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
#include "odometry.h"
#include "go_to_angle.h"
using namespace Pololu3piPlus32U4;

#define DEAD_RECKONING false
const float diaL = 3.2; 
const float diaR = 3.2;
const int nL = 12;
const int nR = 12;
const float w = 9.6;
const int gearRatio = 75;
const double minOutput = -100;
const double maxOutput = 100;
const float kp = 60;  // Adjust values for these controller gains
const float kd = 80;

GoToAngle::GoToAngle(float kp, float kd, double minOutput, double maxOutput)
    : _pdController(kp, kd, minOutput, maxOutput), 
      _odom(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING)
      
{

}

double GoToAngle::update(double target_angle) {
  float current_angle = _odom.get_angle();  // Takes angle data from Odometry get_angle function.
  float error = target_angle - current_angle;
  Serial.println(target_angle);
  Serial.println(current_angle);
  float tolerance = 3.0;  // Tolerance of 3 degrees

  if (abs(error) > tolerance) {
    if (error > 0) {
      _motors.setSpeeds(0, 100);  // Turn right
    } else {
      _motors.setSpeeds(0, -100);  // Turn left
    }
  } else {
    _motors.setSpeeds(0, 0);  // Stop when within tolerance
  }

  return error;  // Return the remaining error
}

