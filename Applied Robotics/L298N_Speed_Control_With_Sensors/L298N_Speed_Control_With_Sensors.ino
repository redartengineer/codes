//Connect to COM5 for Bluetooth with HC-05

#include <Servo.h>
#include "SR04.h"
#define TRIG_PIN_F 23
#define ECHO_PIN_F 22
SR04 sr04F = SR04(ECHO_PIN_F,TRIG_PIN_F);
long FRONT_SENSE;

#define TRIG_PIN_L 27
#define ECHO_PIN_L 26
SR04 sr04L = SR04(ECHO_PIN_L,TRIG_PIN_L);
long LEFT_SENSE;


#define TRIG_PIN_R 29
#define ECHO_PIN_R 28
SR04 sr04R = SR04(ECHO_PIN_R,TRIG_PIN_R);
long RIGHT_SENSE;



Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

int dir1Pin1A = 2; // direction
int dir1Pin2A = 3; // direction
int speedPinA = 6;  // pin ENA on the motor shield. Controls speed.

int dir2Pin1A = 4; // direction
int dir2Pin2A = 5; // direction
int speedPinB = 7;  // pin ENA on the motor shield. Controls speed.


unsigned long time; 
int speed; 
int dir; 

void setup () //Declares pinMode function. 
              //Declares Serial Channel for communication.
{
  myservo.attach(24);  // attaches the servo on pin 9 to the servo object
  pinMode (dir1Pin1A, OUTPUT); 
  pinMode (dir1Pin2A, OUTPUT); 
  pinMode (speedPinA, OUTPUT); // SPEED A
  pinMode (speedPinB, OUTPUT); // SPEED A
  Serial.begin (9600); //Communicates baud rate.
                       //PC communicates with arduino in channel 9600.
} 

//------------------------Start User Defined Library
void forward() //Goes forward. Type void forward() to call function.
              //Make sure to insert 'delay(???)' of choice after calling.
{
  Serial.println ("Motor: Forward");
  analogWrite (speedPinA, 60); // change the number from 1-255 to get the speed you want
  digitalWrite (dir1Pin1A, LOW); //  pin 10 Low, pin 11 High
  digitalWrite (dir1Pin2A, HIGH); // change between LOW and HIGH  

  analogWrite (speedPinB, 60); // change the number from 1-255 to get the speed you want
  digitalWrite (dir2Pin1A, HIGH); //  pin 10 Low, pin 11 High
  digitalWrite (dir2Pin2A, LOW); // change between LOW and HIGH 
}

void back() //Goes backwards. Type void forward() to call function.
              //Make sure to insert 'delay(???)' of choice after calling.
{
  Serial.println ("Motor: Backward");
  analogWrite (speedPinA, 60); // change the number from 1-255 to get the speed you want
  digitalWrite (dir1Pin1A, HIGH); //  pin 10 Low, pin 11 High
  digitalWrite (dir1Pin2A, LOW); // change between LOW and HIGH  

  analogWrite (speedPinB, 60); // change the number from 1-255 to get the speed you want
  digitalWrite (dir2Pin1A, LOW); //  pin 10 Low, pin 11 High
  digitalWrite (dir2Pin2A, HIGH); // change between LOW and HIGH   
}

void left() //Goes Left. Type void forward() to call function.
              //Make sure to insert 'delay(???)' of choice after calling.
{
  Serial.println ("Motor: LEFT");
  analogWrite (speedPinA, 255); // change the number from 1-255 to get the speed you want
  digitalWrite (dir1Pin1A, LOW); //  pin 10 Low, pin 11 High
  digitalWrite (dir1Pin2A, HIGH); // change between LOW and HIGH  
        
  analogWrite (speedPinB, 100); // change the number from 1-255 to get the speed you want
  digitalWrite (dir2Pin1A, LOW); //  pin 10 Low, pin 11 High
  digitalWrite (dir2Pin2A, HIGH); // change between LOW and HIGH 
}

void right() //Goes Right. Type void forward() to call function.
              //Make sure to insert 'delay(???)' of choice after calling.
{
  Serial.println ("Motor: RIGHT");
  analogWrite (speedPinA, 255); // change the number from 1-255 to get the speed you want 255
  digitalWrite (dir1Pin1A, HIGH); //  pin 10 Low, pin 11 High
  digitalWrite (dir1Pin2A, LOW); // change between LOW and HIGH  

  analogWrite (speedPinB, 100); // change the number from 1-255 to get the speed you want. 80
  digitalWrite (dir2Pin1A, HIGH); //  pin 10 Low, pin 11 High
  digitalWrite (dir2Pin2A, LOW); // change between LOW and HIGH   
}

void stop_wheel()
{
  Serial.println ("Motor: OFF");
  analogWrite (speedPinA, 0); // change the number from 1-255 to get the speed you want
  digitalWrite (dir1Pin1A, 0); //  pin 10 Low, pin 11 High
  digitalWrite (dir1Pin2A, 0); // change between LOW and HIGH  

  analogWrite (speedPinB, 150); // change the number from 1-255 to get the speed you want
  digitalWrite (dir2Pin1A, 0); //  pin 10 Low, pin 11 High
  digitalWrite (dir2Pin2A, 0); // change between LOW and HIGH 
}





//------------------------End User Defined Library



//------------------------Start of Main Code
void loop () 
{//Start of Void loop ()
  int input; //number we input to control directions.
          //1 backwards, 2 forwards, 3 left, 4 right, 0 Stop
  
    myservo.write(90); //Servo is at default at angle 90. Faces forward.

   FRONT_SENSE=sr04F.Distance(); //Front HC-SR04
   Serial.print(FRONT_SENSE);
   Serial.println("cm");
   delay(10);

   LEFT_SENSE=sr04L.Distance(); //Left HC-SR04
   //Serial.print("Left:"); //Unlock if want to see distance.
   //Serial.print(LEFT_SENSE); //Unlock if want to see distance.
   //Serial.println("cm");
   delay(10);

   RIGHT_SENSE=sr04R.Distance(); //Right HC-SR04
   //Serial.print("Right:"); //Unlock if want to see distance.
   //Serial.print(RIGHT_SENSE); //Unlock if want to see distance.
   //Serial.println("cm");
   delay(10);
   
  if (FRONT_SENSE >= 41)
  {
  Serial.print(FRONT_SENSE);
  Serial.println("cm");
  forward();
  }

  if (LEFT_SENSE <= 40) //If an object is too close to the Left, goes Right
  {
  Serial.println("Left too close.");
  Serial.print(LEFT_SENSE);
  Serial.println("cm");
  Serial.println("Go Right.");
  right();
  delay(6000);
  stop_wheel();
  delay(1000);
  }

  if (RIGHT_SENSE <= 40) //If an object is too close to the Right, goes Left
  {
  Serial.println("Right too close.");
  Serial.print(RIGHT_SENSE);
  Serial.println("cm");
  left();
  delay(6000); //900 works
  stop_wheel();
  delay(1000);
  }

  //-----If Object too Close Infront
  if (FRONT_SENSE <= 40)
  {//Start of if (pos <= 90) body.
  Serial.println("Front Too Close. Comparing Left and Right.");
  Serial.print(FRONT_SENSE);
  Serial.println("cm");
  Serial.println ("Motor: OFF");
  delay(50);
  stop_wheel();
  delay(1000);  //  continue for 1 second
  back();
  delay(1600);
  stop_wheel();
  
  if (LEFT_SENSE > RIGHT_SENSE) //If an object is too close to the Left, goes Right
  {//Start of LEFT_SENSE > RIGHT_SENSE
  Serial.println("Right:");
  Serial.print(RIGHT_SENSE);
  Serial.println("cm");
  Serial.println("Go Left.");
  left();
  delay(6000); //900 works
  stop_wheel();
  delay(3000);
  }//End of LEFT_SENSE > RIGHT_SENSE
  
  else if (LEFT_SENSE < RIGHT_SENSE) //If an object is too close to the Right, goes Left
  {//Start of LEFT_SENSE < RIGHT_SENSE
  Serial.println("Left:");
  Serial.print(LEFT_SENSE);
  Serial.println("cm");
  Serial.println("Go Right.");
  right();
  delay(6000);
  stop_wheel();
  delay(1000);
  }//End of LEFT_SENSE < RIGHT_SENSE
  }//End of if (pos <= 90) body.

  forward(); //Exits If Statement and continues forward.
  //delay (1000);


//---Start of Manual Controls
if(Serial.available() > 0)
{//start of if Statement Serial.available(). IF it's more than 0, it executes.
    input = Serial.read(); //What we input in the Serial Monitor redirects us 
                          //to case.
    switch(input)
    {//Start of switch-case statement
   
      case '1': //Goes Foward
      {//Start of case FORWARD
      forward();
      delay(3000);  //  continue for 1 second  
      break;
      }//End of case FORWARD

      case '2': //Goes Back
      {//Start of case Back
      back();
      delay(3000);  //  continue for 1 second
      break;
      }//End of case Back

      case '3': //Goes Left
      {//Start of case Left
      left();
      delay(3000);  //  continue for 1 second  
      break;  
      }//End of case Left

      case '4': //Goes Right
      {//Start of case Right
      right();
      delay(3000);  //  continue for 1 second
      break;
      }//End of case Right

      case '0': //Stops motors
      {//Start of case OFF
      stop_wheel();
      delay(3000);  //  continue for 1 second
      break;  
      }//End of case OFF

      
    }//End of switch-case statement
  }//End of if Statement Serial.available().
//---End of Manual Controls


  
} //Start of Void loop ()
//------------------------End of Main Code
