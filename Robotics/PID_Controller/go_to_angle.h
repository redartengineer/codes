#ifndef GO_TO_ANGLE_H
#define GO_TO_ANGLE_H
#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
#include "odometry.h"
using namespace Pololu3piPlus32U4;

class GoToAngle{
  public:
    GoToAngle(float kp, float kd, double minOutput, double maxOutput);
    double update(double target_angle); //Updates the target angle
    
  private:
    PDcontroller _pdController;
    Odometry _odom;             
    Motors _motors; 

};

#endif
