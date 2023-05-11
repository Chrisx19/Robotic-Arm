#include <SoftwareSerial.h>
#include <math.h>

#include <AccelStepper.h>
#include <VarSpeedServo.h>

VarSpeedServo wrist_servo;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // value read from the pot
     // value output to the PWM (analog out)
    
// void min_max(int min_input, int min_output, int min, int max);

void setup()
{
    Serial.begin(9600);
    wrist_servo.attach(9);
}

void loop()
{
  sensorValue = analogRead(analogInPin)/ (680/100);

  float degree_conversion = map(sensorValue, 26, 89, -90, 90);

  if (sensorValue > 89)
  {
    degree_conversion = 90;
  }
  else if (sensorValue < 26)
  {
    degree_conversion = -90;
  }

  int servo_command = 0;
  float servo_conversion = map(servo_command, -90, 90, 37, 145);

  wrist_servo.write(servo_conversion, 10);

  Serial.print("       Raw = ");
  Serial.print(sensorValue);
  Serial.print("       Degree = ");
  Serial.println(degree_conversion);
  delay(100);
}

// void min_max(int min_input, int min_output, int min, int max)
// {
//   if (sensorValue > 89)
//   {
//     degree_conversion = 90;
//   }
//   else if (sensorValue < 26)
//   {
//     degree_conversion = -90;
//   }
// }