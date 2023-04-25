#include <SoftwareSerial.h>
#include <math.h>

#include <AccelStepper.h>
#include <VarSpeedServo.h>

VarSpeedServo myservo;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // value read from the pot
     // value output to the PWM (analog out)

void setup()
{
    Serial.begin(9600);
    myservo.attach(9);
}

void loop()
{
  sensorValue = analogRead(analogInPin)/ (680/100);

  float degree_conversion = map(sensorValue, 3, 35, 0, 90);

  if (sensorValue > 35)
  {
    degree_conversion = 90;
  }
  else if (sensorValue < 3)
  {
    degree_conversion = 0;
  }

  int servo_command = 0;
  float servo_conversion = map(servo_command, -90, 90, 0, 55);

//  myservo.writeMicroseconds(2055);  //900 left 1500 middle  2055 right

  myservo.write(90, 15); //37 left 90 middle  142 right



  Serial.print("       Raw = ");
  Serial.print(sensorValue);
  Serial.print("       Degree = ");
  Serial.println(degree_conversion);
  delay(100);



}
