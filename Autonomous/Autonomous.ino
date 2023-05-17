#include <AccelStepper.h>
#include <VarSpeedServo.h>
#include <math.h>

#define waist_pin      A0
#define shoulder_pin   A1
#define elbow_pin      A2
#define wrist_pin      A3
#define wrist_roll_pin A4

AccelStepper    waist_joint1(1, 2, 3), shoulder_joint2(1, 4, 5); //pulse //dir
VarSpeedServo   elbow_joint3, wrist_joint4, wrist_roll_joint5;

int16_t waist_val_raw      = 0,
        shoulder_val_raw   = 0,
        elbow_val_raw      = 0,
        wrist_val_raw      = 0,
        wrist_roll_val_raw = 0;        // value read from the pot
        

int8_t  waist_cmd      = 0,
        shoulder_cmd   = 0,
        elbow_cmd      = 0,
        wrist_cmd      = 0,        // value output to the PWM (analog out)
        wrist_roll_cmd = 0;


const int Joints_EN_Pin = 13;
//*****************************************FUNCTION CALLER*****************************
void read_analog(void);
void max_min(int val_raw, int low, int high);
void stepper_position(int target, int sensor, int velocity,int threshold);



//#################################BEGIN SETUP  FUNCTION################################
void setup() {
  
  Serial.begin(9600);
  waist_joint1.setMaxSpeed(1000000);
  shoulder_joint2.setMaxSpeed(1000000);

  elbow_joint3.attach(6);
  wrist_joint4.attach(7);       
  wrist_roll_joint5.attach(8);

  waist_cmd      = 0;
  shoulder_cmd   = 0;
  elbow_cmd      = 0;
  wrist_cmd      = 0;
  wrist_roll_cmd = 0;
}
//#################################END SETUP  FUNCTION################################

//#################################BEGIN LOOOP  FUNCTION################################
void loop() {
  // read the value from the sensor:
  read_analog();
  
  waist_joint1.runSpeed();
  shoulder_joint2.runSpeed();

  uint8_t joint_adc_min[5] = {0, 0, 36, 32, 34};
  uint8_t joint_adc_max[5] = {0, 0, 129, 127, 133};

  uint8_t elbow =      map(elbow_cmd,       90, -90, 36, 140);          //min 35(servo write cmd)~ -90 degree ; max 140~ 90 degree/////////////MID ~ 88
  uint8_t wrist =      map(wrist_cmd,      -90, 90, 34, 143);          //min 34~ -90 degree ; max 143~ 90 degree
  uint8_t wrist_roll = map(wrist_roll_cmd, -90, 90, 35, 145);          //min 35~ -90 degree ; max 135~ 90 degree

  max_min(waist_val_raw,      joint_adc_min[0], joint_adc_max[0]);          //raw_val from ADC, minimum val_adc, maximum val_adc
  max_min(shoulder_val_raw,   joint_adc_min[1], joint_adc_max[1]);          
  max_min(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2]);
  max_min(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3]);          
  max_min(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4]);

  int8_t  elbow_degree =      map(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2],  90, -90); 
  int8_t  wrist_degree =      map(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3], -90, 90);
  int8_t  wrist_roll_degree = map(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4], -90, 90);

//*********************************Motor Commands***********************************
  stepper_position(waist_val_raw, waist_cmd, 500 ,10);
  stepper_position(shoulder_val_raw, shoulder_cmd, 500 ,10);
  elbow_joint3.slowmove(elbow, 15);
  wrist_joint4.slowmove(wrist, 15);
  wrist_roll_joint5.slowmove(wrist_roll, 15);
//*********************************Motor Commands***********************************

  //  Serial.print("       Raw_val_prot = ");       //debug for Testing 
  //  Serial.print(wrist_val_raw);

//  Serial.print("       elbow = ");
//  Serial.print(elbow_degree);
//
//  Serial.print("       wrist  = ");
//  Serial.print(wrist_degree);
//  
//  Serial.print("       wrist_roll = ");
//  Serial.println(elbow_degree);
//  delay(100);

}
//#################################END LOOOP  FUNCTION################################


//#######################START OF FUNCTION IMPLEMENT##################################
double g_decel = 500;
void stepper_position(int sensor, int target, int velocity,int threshold)
{
  
  if (sensor > (target - threshold) && sensor < (target + threshold))
  {
    g_decel /= 1.081;
    if (g_decel < 10)
    {
      g_decel = 0;
    }
    waist_joint1.setSpeed(g_decel);
    delay(30);
  }
  else
  {
    g_decel = velocity;
    waist_joint1.setSpeed(velocity);
  }
//    Serial.print(g_decel);      //Debug for value decel
//    Serial.print("    ");
//    Serial.println(sensor);
}

// Servo_pit_feed
void max_min(int val_raw, int low, int high)
{
  if ( val_raw <= low) {
    val_raw = low;
  }
  else if (val_raw >= high){
    val_raw = high;
  }
}

uint16_t g_target_adc_raw[5] = {500, 500, 155, 155, 155};
void read_analog(void)
{ 
  //*********************************Stepper Analog***********************************
  waist_val_raw      = analogRead(waist_pin) / (1023 / g_target_adc_raw[0]);
  shoulder_val_raw   = analogRead(shoulder_pin) / (1023 / g_target_adc_raw[1]);
  //*********************************Stepper Analog***********************************
  
  //*********************************Servo Analog***********************************
  elbow_val_raw      = analogRead(elbow_pin) / (685 / g_target_adc_raw[2]);
  wrist_val_raw      = analogRead(wrist_pin) / (685 / g_target_adc_raw[3]);  //685 max
  wrist_roll_val_raw = analogRead(wrist_roll_pin) / (685 / g_target_adc_raw[4]);
  //*********************************Servo Analog***********************************

//  *********************************Debug for Analog***********************************
//  Serial.print(waist_val_raw);      
//  Serial.print("\t");
//  Serial.print(shoulder_val_raw);      
//  Serial.print("\t");
//  Serial.print(elbow_val_raw);      
//  Serial.print("\t");
//  Serial.print(wrist_val_raw); 
//  Serial.print("\t");
//  Serial.println(wrist_roll_val_raw);  

}
