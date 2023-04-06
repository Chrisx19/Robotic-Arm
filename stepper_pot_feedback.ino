#include <AccelStepper.h>

AccelStepper stepper1(1, 3, 2);

int pos = 5; //set position value, 0 -> 10 
int speed = 300; // speed of stepper

void setup() {
  stepper1.setMaxSpeed(10000);
  //Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(A0) / 100; //potValue is the value of the pot read by arduino divided by 100, 0 -> 10
  delay(1);

  while(pos>potValue) //if pos>potValue, stepper rotates ccw until pos==potValue
  {
    potValue = analogRead(A0) / 100;
    delay(1);
    stepper1.setSpeed(-1 * speed); //negative value means stepper rotates ccw
    stepper1.runSpeed();

    if(pos==potValue)
    {
      break;
    }
  }

  while(pos<potValue) //if pos<potValue, stepper rotates cw until pos==potValue
  {
    potValue = analogRead(A0) / 100;
    delay(1);
    stepper1.setSpeed(speed); //positive value means stepper rotates cw
    stepper1.runSpeed();
    
    if(pos==potValue)
    {
      break;
    }
  }
}
