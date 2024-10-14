#ifndef My_Robot_h
#define My_Robot_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

extern Motors motors;  // Declare motors as extern

class MyRobot{
  public:
    MyRobot(float base_speed);
    void forward(float distance);
    void backward(float distance);
    void halt();
    void turn_left(float duration);
    void turn_right(float duration);
    
  private:
    float _base_speed;
};

#endif
