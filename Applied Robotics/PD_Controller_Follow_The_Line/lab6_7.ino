#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 0.06
#define kd_line 0.5
#define kp_obs 50
#define kd_obs 20

int calibrationSpeed = 120;                                   //Sets the calibration reading to 120
unsigned int lineSensorValues[5];                             //Takes values from index 5, from array
PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput); //PD controller for following the line
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);    //PD controller for avoiding obstacles


void calibrateSensors()                                      //Function to calibrate readings
{
  //TODO: write function to calibration IR line sensors
  for(int i = 0; i < 80; i++) {                              //For loop serving as a timer. Resets after it reaches 80 counters.
    /*if (i <= 20) {
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }*/
    if(i > 20 && i < 60) {                                   //If the counter is less than 20 and 60, turn left.
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    }
    else if(i >= 60) {                                       //If the counter is more than or equal to 60, turn right. 
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0,0);                                     //Stops the motors.
}

void setup() {
  //Serial.begin(9600);
  //servo.attach(5);
  //servo.write(90); // turn servo forward
  //delay(2000);

  calibrateSensors();                                       //Calls function
  
}

void loop(){

 //TODO: main code here
 //Hint: may need additional functions
  int d_position = 2000;                                            //Desired position
  int a_position = lineSensors.readLineBlack(lineSensorValues);     //Actual Position
  int PDout = pd_line.update(a_position, d_position);               //Takes the error between the current position minus the desired position
  motors.setSpeeds(int(baseSpeed + PDout), int(baseSpeed - PDout)); //Sets the motors based on the calculated errors
  //Serial.println(lineSensors.read(lineSensorValues));
    
}

void placeholder() {

}


