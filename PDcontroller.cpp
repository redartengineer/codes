#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(int kp, int kd, double minOutput, double maxOutput) {
  _kp = kp;
	_kd = kd;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
	_error = 0.0;
  _lastError = 0.0;
	_output = 0.0;
	_clampOut = 0.0;
	_previousTime = 0.0;
	_currentTime = millis();
}

double PDcontroller::update(double value, double target_value){
  _error = value - target_value;
  
  _currentTime = millis();
  
  double p, d;
  
  d = _kp*_error;  

  if (_previousTime!=0.0){ // P component + D component
	  p = _kd*(_error - _lastError)/(_currentTime-_previousTime);
  }
  else{ // at the begining, we only have P component.
    p = 0.0;
  }
  
  _output = p + d;
  _clampOut = constrain(_output, _minOutput, _maxOutput);
  
  _previousTime = _currentTime;
  _lastError = _error;

  return _clampOut;
}
