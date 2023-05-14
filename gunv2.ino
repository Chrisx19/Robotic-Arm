#include <Servo.h>

Servo myservo; 

int pos = 0;
int servoPos = 0;
const int buttonOne = 12; //actuator
const int buttonTwo = 13; //servo position

void setup() {
pinMode(10, OUTPUT); //trigger
pinMode(9, OUTPUT); //safety
pinMode(buttonOne, INPUT_PULLUP);
pinMode(buttonTwo, INPUT_PULLUP);
myservo.attach(8);

//sets initial servo position to 0 deg
for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  delay(15);                       // waits 15ms for the servo to reach the position
}
}

void loop() {
  bool button1 = digitalRead(buttonOne);
  bool button2 = digitalRead(buttonTwo);

  if(button1==false)
  {
    //bool button1 = digitalRead(buttonOne);
    digitalWrite(9,HIGH); //activate safety
    //delay(500);
    digitalWrite(10, HIGH); //activate trigger
    delay(1);

    digitalWrite(9, LOW); //disable safety
    digitalWrite(10, LOW);  //disable trigger
  }

  if(button2==false)
  {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);
      servoPos = 0;                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      servoPos = 1;
    }
  }
}

