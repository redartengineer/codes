#include <Pololu3piPlus32U4.h>
//#include <Wire.h>
#include "printOLED.h"
#include "odometry.h"
#include "PDcontroller.h"
#include "PID_controller.h"
#include "go_to_angle.h"

using namespace Pololu3piPlus32U4;

Motors motors; //Allows use of motors in code
Encoders encoders; //Allows use of encoders in code

#define DEAD_RECKONING false
#define diaL 3.2 //Diameter of Left wheel from documentation
#define diaR  3.2 //Diameter of Right wheel from documentation
#define nL 12 //Ticks of Left wheel from documentation
#define nR 12 //Ticks of Right wheel from documentation
#define w 9.6 //Width of wheelfrom documentation
#define gearRatio 75 //Ratio of Gear from documentation
#define minOutput -100 //Clamped minimum to restrict output
#define maxOutput 100 //Clamped maximum to restrict output
#define kp 5 //Proportional Gain
#define kd 10 // Derivative Gain
#define ki 60 // Integral Gain

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PDcontroller pdcontroller(kp, kd, minOutput, maxOutput); //Declares the pdcontroller
PIDcontroller pidcontroller(kp, kd, ki, minOutput, maxOutput); //Declares the pidcontroller
GoToAngle goToAngle(kp, kd, minOutput, maxOutput); //Declares the goToAngle function

// goals in cm and rad
const float goal_x = 0.5; 
const float goal_y = 0.5;
const float goal_theta =0.8; 

//odometry
int16_t deltaL=0, deltaR=0; //Change in Left and Change in right
int16_t encCountsLeft = 0, encCountsRight = 0; //Gives blank slate to left and right encoders
float x, y, theta; //Declares variables

void setup() {
  Serial.begin(9600);
}

void loop() {
  //Obtain time

  // Read data from encoders
  deltaL = encoders.getCountsAndResetLeft(); //Gets the counts of the left wheel and resets.
  deltaR = encoders.getCountsAndResetRight(); //Gets the counts of the right wheel and resets.

  // Increment total encoder cound
  encCountsLeft += deltaL; //Increments left encoder count
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta); //calculate robot's position
  double error = pidcontroller.update(theta, (3.1415)); //Takes the error between the current angle minus the desired angle
  motors.setSpeeds(error, -1 * error); //Sets the motors based on the calculated errors

  //Serial.print("Error: ");
  //Serial.println(error);
  //delay(50);

  /*if(odometry.get_angle() == target_angle) {
    motors.setSpeeds(0,0);
  }*/

  // TODO call controller's update function
  // and the motors.setSpeeds(left, right) here


}
