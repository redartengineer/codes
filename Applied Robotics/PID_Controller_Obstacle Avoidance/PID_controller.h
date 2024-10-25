#ifndef PID_controller_h
#define PID_controller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(int kp, int kd, int ki, double minOutput, double maxOutput); //Calls values of variables
    double update(double value, double target_value); //Updates value and target_value
    
  private: //Declares variables
    int _kp; 
	  int _kd;
    int _ki;
    double _maxOutput;
	  double _minOutput;
	  double _lastError;
	  double _error;
	  double _output;
	  double _clampOut;
   long int _previousTime;
   long int _currentTime;
};

#endif
