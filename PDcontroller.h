#ifndef PDcontroller_h
#define PDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PDcontroller{
  public:
    PDcontroller(int kp, int kd, double minOutput, double maxOutput);
    double update(double value, double target_value);
    
  private:
    int _kp;
	  int _kd;
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
