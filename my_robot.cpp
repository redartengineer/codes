#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;


MyRobot::MyRobot(float base_speed) {
  _base_speed = base_speed/1000;
}

void MyRobot::halt(){
  motors.setSpeeds(0, 0);  
}

void MyRobot::forward(float distance) { //input: "distance"

  float duration = (distance / _base_speed) * 1000.0;   //duration is in MilliSeconds

  motors.setSpeeds(int(_base_speed * 1000), int(_base_speed * 1000));

  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long intDuration = (unsigned long)duration;
  
  while (elapsedTime < intDuration) {
    elapsedTime = millis() - startTime;
    // Serial.print("Elapsed time: ");
    // Serial.println(elapsedTime);
  }
  halt(); // After driving for the specified "distance", then stop the car.
}

void MyRobot::backward(float distance){ //input: "distance"
  motors.setSpeeds(-int(_base_speed*1000), -int(_base_speed*1000));
  float duration = distance/_base_speed*1000;
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long intDuration = (unsigned long)duration;
  while(elapsedTime<duration){
    elapsedTime=millis()-startTime;
    // Serial.print("Elapsed time: ");
    // Serial.println(elapsedTime);
  }
  halt(); // After driving for the specified "distance", then stop the car.
}


void MyRobot::turn_left(float duration){ //input: "duration"
  motors.setSpeeds(-int(_base_speed*1000), int(_base_speed*1000));
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long intDuration = (unsigned long)duration;
  while(elapsedTime<intDuration){
    elapsedTime = millis()-startTime;
    // Serial.print("Elapsed time: ");
    // Serial.println(elapsedTime);
  }
  halt(); // After driving for the specified "duration", then stop the car.
}

void MyRobot::turn_right(float duration){ //input: "duration"
  motors.setSpeeds(int(_base_speed * 1000), -int(_base_speed * 1000));

  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long intDuration = (unsigned long)duration;
  while (elapsedTime < intDuration) {
    elapsedTime = millis() - startTime;
    // Serial.print("Elapsed time: ");
    // Serial.println(elapsedTime);
  }
  halt(); // After driving for the specified "duration", then stop the car.
}
