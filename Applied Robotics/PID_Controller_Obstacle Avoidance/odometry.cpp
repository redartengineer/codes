#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
#include "printOLED.h"

using namespace Pololu3piPlus32U4;

#define PI 3.14159

PrintOLED printOLED;

Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;
  _deadreckoning = dead_reckoning;

  _x = 0;
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;

  if(_deadreckoning){ // if using dead reckoning, initialize and calibrate IMU
    Wire.begin();
    _imu.init();
    _imu.enableDefault();

    //calibrate IMU
    int total = 0;
    for (int i = 0; i < 100; i++)
    {
      _imu.readGyro();
      total += _imu.g.z;
      delay(1);
    }
    _IMUavg_error = total / 100;  
  }
}

void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta){
	// do odometry math to calculate robot's new position and orientation

double deltaL = (double)((left_encoder_counts - _left_encoder_counts_prev)*PI*_diaL)/(_nL *_gearRatio);
double deltaR = (double)((right_encoder_counts - _right_encoder_counts_prev)*PI*_diaR)/(_nR *_gearRatio); 
_theta += (deltaL - deltaR) / _w; // Calculates theta
_theta = theta;


  if(_deadreckoning){ // if using dead reckoning, get angle from IMU
      _imu.readGyro();
      float angleRate = (_imu.g.z - _IMUavg_error);
      _theta += angleRate * 0.0001;
  }
  else{ // otherwise, calculate angle from encoders using the formula on the lecture slides
    _theta += (deltaR-deltaL)/_w;
  }

  _x = ((float)((float)(deltaL+deltaR)/2) * cos(_theta)); //update x   // _theta passed to cos function in radians
  _y = ((float)((float)(deltaL+deltaR)/2) * sin(_theta)); //update y   // _theta passed to sin function in radians

  x += _x;
  y += _y;
  theta = _theta;


  printOLED.print_odom(x,y,theta);

  Serial.print("theta:  "); //reminder: 90 degrees == Pi/2 == 1.57
  Serial.println(_theta);

  Serial.print("delta x:  ");
  Serial.println(_x);
  Serial.print("delta y:  ");
  Serial.println(_y);

  Serial.print("total x:  ");
  Serial.println(x);
  Serial.print("total y:  ");
  Serial.println(y);
  
  // Save the values as the "previous" values, so you can use it in the next iteration
  _left_encoder_counts_prev = left_encoder_counts;
  _right_encoder_counts_prev = right_encoder_counts;
  delay(10);
	
  //return _theta;
}

float Odometry::get_angle(){ //Returns value of theta
  return _theta;
}
