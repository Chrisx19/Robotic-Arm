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

  float degree_conversion = map(sensorValue, 12, 106, 0, 90);

  if (sensorValue > 106)
  {
    degree_conversion = 90;
  }
  else if (sensorValue < 12)
  {
    degree_conversion = 0;
  }

  int servo_command = 45;
  float servo_conversion = map(servo_command, 0, 90, 0, 55);

  myservo.write(servo_conversion, 20);



  Serial.print("       Raw = ");
  Serial.print(sensorValue);
  Serial.print("       Degree = ");
  Serial.println(degree_conversion);
  delay(100);



}