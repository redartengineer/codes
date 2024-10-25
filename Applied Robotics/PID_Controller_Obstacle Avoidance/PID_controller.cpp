#include <Pololu3piPlus32U4.h>
#include "PID_controller.h"
using namespace Pololu3piPlus32U4;

//PIDController in charge of calculating the output with the use of proportional, derivative, and integral gains
PIDcontroller::PIDcontroller(int kp, int kd, int ki, double minOutput, double maxOutput) { //Extracts values from lab5.ino
  _kp = kp;
	_kd = kd;
  _ki = ki;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
	_error = 0.0; //Declares as 0 for clean state
  _lastError = 0.0; //Declares as 0 for clean state
	_output = 0.0; //Declares as 0 for clean state
	_clampOut = 0.0; //Declares as 0 for clean state
	_previousTime = 0.0; //Declares as 0 for clean state
	_currentTime = millis(); //Obtains the current time in milliseconds
}

double PIDcontroller::update(double value, double target_value){ //Extracts current value and target value and updates them
  _error = value - target_value; //Equation to obtain the error from the proportional gain
  
  _currentTime = millis(); //Obtains the current time in milliseconds
  double p, d, i; //Declares variables
  
  d = _kp*_error; //Equation to obtain the error from the derivative gain

  _error += _error*(_previousTime-_currentTime)/1000; //Sums the error through time.
  i = _ki * (_error); //Equation to obtain the error from the integral gain

  if (_previousTime!=0.0){ // P component + D component
	  p = _kd * (_error - _lastError)/(_currentTime-_previousTime);
  }
  else{ // at the begining, we only have P component.
    p = 0.0;
  }
  
  _output = p + d + constrain(i, _minOutput, _maxOutput); //Output is summed form all the PID errors. Accumulated error form ki is clamped independently to avoid growing large.
  _clampOut = constrain(_output, _minOutput, _maxOutput); // Output is clamped
  
  _previousTime = _currentTime; //Previoustime obtains current time value
  _lastError = _error; //Last Error is error value

  return _clampOut; //Reyurns value of clampout
}