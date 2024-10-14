#include <Pololu3piPlus32U4.h>
#include "Pcontroller.h"
using namespace Pololu3piPlus32U4;

Pcontroller::Pcontroller(float kp, double minOutput, double maxOutput) {
  _kp = kp;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
	_error = 0.0;
  
	_output = 0.0;
	_clampOut = 0.0;
}

double Pcontroller::update(double value, double target_value){
  _error = value - target_value;
 
	_output = _kp*_error; // P component
  
  _clampOut = constrain(_output, _minOutput, _maxOutput);

  return _clampOut;
}
