#include <SoftwareSerial.h>
#include <math.h>

#include <AccelStepper.h>
#include <VarSpeedServo.h>

VarSpeedServo servo_elbow;
int degree = 0;

//const int analogElbowPin = A3;
const int analogWristPin = A0;  // Analog input pin that the potentiometer is attached to

int elbow_val = 0,
    wrist_val = 0;        // value read from the pot
     // value output to the PWM (analog out)

void setup()
{
    Serial.begin(9600);
    servo_elbow.attach(9);
    
}

void loop()
{
//  elbow_val = analogRead(analogElbowPin)/ (680/100);
  wrist_val = analogRead(analogWristPin);

  degree = map(wrist_val, 7, 106, 0, 180);


  Serial.print("  Wrist Raw = ");
  Serial.print(wrist_val);

  

  Serial.print("  Deg = ");
  Serial.println(degree);

  servo_elbow.write(90);
  delay(100);



}
