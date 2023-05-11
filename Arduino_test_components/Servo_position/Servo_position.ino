#include <Servo.h>
Servo myservo;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to


int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int degree = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
    myservo.attach(9);
}

void loop() {

  sensorValue = analogRead(analogInPin);

  outputValue = map(sensorValue, 33, 429, 0, 180);

  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("       Degree = ");
  Serial.println(outputValue);

  uint8_t const setpoint_position = 177;
  uint8_t servo_conversion = setpoint_position /1.5;
    
    myservo.write(servo_conversion);              // tell servo to go to position in variable 'pos'

}
