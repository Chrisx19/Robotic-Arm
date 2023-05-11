#include <SoftwareSerial.h>
#include <math.h>

#include <AccelStepper.h>
#include <VarSpeedServo.h>

VarSpeedServo elbow_joint3, wrist_joint4, wrist_roll_joint5;

const int elbow_pin      = A2,
          wrist_pin      = A3,
          wrist_roll_pin = A4;// Analog input pin that the potentiometer is attached to

int16_t elbow_val_raw = 0,
        wrist_val_raw = 0,
        wrist_roll_val_raw = 0;        // value read from the pot
        

int8_t  elbow_cmd  = 0,
        wrist_cmd  = 0,        // value output to the PWM (analog out)
        wrist_roll_cmd = 0;


void read_analog(void);
void max_min(int val_raw, int low, int high);

void setup()
{
    Serial.begin(9600);
  
    elbow_joint3.attach(6);
    wrist_joint4.attach(7);       
    wrist_roll_joint5.attach(8);
    
//    elbow_joint3.detach();
//    wrist_joint4.detach();
//    wrist_roll_joint5.detach();

  elbow_cmd      = 0;
  wrist_cmd      = 0;
  wrist_roll_cmd = 0;
}

void loop()
{
  read_analog();
  
  uint8_t joint_adc_min[5] = {0, 0, 36, 32, 34};
  uint8_t joint_adc_max[5] = {0, 0, 129, 127, 133};

  uint8_t elbow =      map(elbow_cmd,       90, -90, 36, 140);          //min 35(servo write cmd)~ -90 degree ; max 140~ 90 degree/////////////MID ~ 88
  uint8_t wrist =      map(wrist_cmd,      -90, 90, 34, 143);          //min 34~ -90 degree ; max 143~ 90 degree
  uint8_t wrist_roll = map(wrist_roll_cmd, -90, 90, 35, 145);          //min 35~ -90 degree ; max 135~ 90 degree

//  max_min(waist_val_raw,      joint_adc_min[0], joint_adc_max[0]);          //raw_val from ADC, minimum val_adc, maximum val_adc
//  max_min(shoulder_val_raw,      joint_adc_min[1], joint_adc_max[1]);          
  max_min(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2]);
  max_min(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3]);          
  max_min(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4]);

  int8_t  elbow_degree =      map(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2],  90, -90); 
  int8_t  wrist_degree =      map(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3], -90, 90);
  int8_t  wrist_roll_degree = map(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4], -90, 90);

  
  elbow_joint3.slowmove(elbow, 15);
  wrist_joint4.slowmove(wrist, 15);
  wrist_roll_joint5.slowmove(wrist_roll, 15);
  

  //  Serial.print("       Raw_val_prot = ");       //debug for Testing 
  //  Serial.print(wrist_val_raw);

  Serial.print("       elbow = ");
  Serial.print(elbow_degree);

  Serial.print("       wrist  = ");
  Serial.print(wrist_degree);
  
  Serial.print("       wrist_roll = ");
  Serial.println(elbow_degree);
  
  delay(100);

}

void max_min(int val_raw, int low, int high)
{
  if ( val_raw <= low) {
    val_raw = low;
  }
  else if (val_raw >= high){
    val_raw = high;
  }
}

void read_analog(void)
{
  
  elbow_val_raw = analogRead(elbow_pin) / (685 / 155);
  wrist_val_raw = analogRead(wrist_pin) / (685 / 155);  //685 max
  wrist_roll_val_raw = analogRead(wrist_roll_pin) / (685 / 155);
}
