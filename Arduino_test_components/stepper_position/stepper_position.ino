#include <AccelStepper.h>

AccelStepper    waist_joint1(1, 2, 3), shoulder_joint2(1,5,6); //pulse //dir

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
const int Joints_EN_Pin = 4;
int target = 389;


void stepper_position(int target, int sensor, int velocity,int threshold);

void setup() {
  // declare the ledPin as an OUTPUT:
  
  Serial.begin(9600);
  waist_joint1.setMaxSpeed(1000000);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin) / (1023/500);
  
  waist_joint1.runSpeed();

  stepper_position(sensorValue, target, 500 ,10);

}

double decel = 500;
void stepper_position(int sensor, int target, int velocity,int threshold)
{
  
  if (sensor > (target - threshold) && sensor < (target + threshold))
  {
    decel /= 1.081;
    if (decel < 10)
    {
      decel = 0;
    }
    waist_joint1.setSpeed(decel);
    delay(30);
  }
  else
  {
    decel = velocity;
    waist_joint1.setSpeed(velocity);
  }
    Serial.print(decel);
    Serial.print("    ");
    Serial.println(sensor);
}
