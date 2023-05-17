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
bool  g_Joints_disable_Joy = false;
//*****************************************FUNCTION CALLER*****************************

void stepper_position_1(int target, int sensor, int velocity,int threshold);
void stepper_position_2(int target, int sensor, int velocity,int threshold);
int max_min(int16_t val_raw, uint16_t lower, uint16_t higher);
void disable_motors(bool motor_EN);
void read_analog(void);


//#################################BEGIN SETUP  FUNCTION################################
void setup() {
  
  Serial.begin(9600);
  pinMode(Joints_EN_Pin, OUTPUT);
  waist_joint1.setMaxSpeed(1000000);
  shoulder_joint2.setMaxSpeed(1000000);

  elbow_joint3.attach(6);
  wrist_joint4.attach(7);       
  wrist_roll_joint5.attach(8);


//  joint[elbow]      = 88;
//  joint[wrist]      = 88.5;
//  joint[wrist_roll] = 90;

  waist_cmd      = 50;
  shoulder_cmd   = -30;
  elbow_cmd      = 0;
  wrist_cmd      = 0;
  wrist_roll_cmd = 0;
  
}
//#################################END SETUP  FUNCTION################################

//#################################BEGIN LOOOP  FUNCTION################################
void loop() {
  // read the value from the sensor:
  read_analog();

  g_Joints_disable_Joy = true;
  disable_motors(g_Joints_disable_Joy);
  
  waist_joint1.runSpeed();
  shoulder_joint2.runSpeed();

  uint16_t joint_adc_min[5] = {0, 268, 36, 32, 34};
  uint16_t joint_adc_max[5] = {0, 302, 129, 127, 133};

//  int16_t waist =      map(waist_cmd,      -30, 90, 302, 36);
  int16_t shoulder =   map(shoulder_cmd,   -30, 90, 302, 268);
  int16_t elbow =      map(elbow_cmd,       90, -90, 36, 140);          //min 35(servo write cmd)~ -90 degree ; max 140~ 90 degree/////////////MID ~ 88
  int16_t wrist =      map(wrist_cmd,      -90, 90, 34, 143);          //min 34~ -90 degree ; max 143~ 90 degree
  int16_t wrist_roll = map(wrist_roll_cmd, -90, 90, 35, 145);          //min 35~ -90 degree ; max 135~ 90 degree

// There was a bug here before when it was using void but not its good with return func
  waist_val_raw      = max_min(waist_val_raw,      joint_adc_min[0], joint_adc_max[0]);          //raw_val from ADC, minimum val_adc, maximum val_adc
  shoulder_val_raw   = max_min(shoulder_val_raw,   joint_adc_min[1], joint_adc_max[1]);        
  elbow_val_raw      = max_min(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2]);
  wrist_val_raw      = max_min(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3]);          
  wrist_roll_val_raw = max_min(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4]);

  int16_t  shoulder_degree =   map(shoulder_val_raw,   joint_adc_min[1], joint_adc_max[1], -30, 90); 
  int16_t  elbow_degree =      map(elbow_val_raw,      joint_adc_min[2], joint_adc_max[2],  90, -90); 
  int16_t  wrist_degree =      map(wrist_val_raw,      joint_adc_min[3], joint_adc_max[3], -90, 90);
  int16_t  wrist_roll_degree = map(wrist_roll_val_raw, joint_adc_min[4], joint_adc_max[4], -90, 90);

//*********************************Motor Commands***********************************
  stepper_position_1(waist_val_raw, waist_cmd, 500 ,10);
  stepper_position_2(shoulder_val_raw, shoulder_cmd, 500 ,10);
  elbow_joint3.slowmove(elbow, 15);
  wrist_joint4.slowmove(wrist, 15);
  wrist_roll_joint5.slowmove(wrist_roll, 15);
//*********************************Motor Commands***********************************


//    Serial.print("       Raw_val = ");       //debug for Testing 
//    Serial.print(shoulder_val_raw);
//    Serial.print("\t");
//    Serial.print("       Degree = ");
//    Serial.println(shoulder_degree);

//  Serial.print("       Test = ");
//  Serial.println(shoulder);
//
//  Serial.print("       wrist  = ");
//  Serial.print(wrist_degree);
//  
//  Serial.print("       Degree = ");
//  Serial.println(waist);
//  delay(100);

}
//#################################END LOOOP  FUNCTION################################


//#######################START OF FUNCTION IMPLEMENT##################################
double g_decel = 500;
void stepper_position_1(int sensor, int target, int velocity,int threshold)
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

void stepper_position_2(int sensor, int target, int velocity,int threshold)
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

// Max_MIN
int max_min(int16_t val_raw, uint16_t lower, uint16_t higher)  
{
  if ( val_raw <= lower) {
    val_raw = lower;
  }
  else if (val_raw >= higher){
    val_raw = higher;
  }
 return val_raw;
}


void disable_motors(bool motor_EN)
{
  if (motor_EN)                    
  {                                      
    digitalWrite(Joints_EN_Pin, LOW);
    elbow_joint3.detach();
    wrist_joint4.detach();
    wrist_roll_joint5.detach();
  }
  else
  {
    digitalWrite(Joints_EN_Pin, HIGH);
    elbow_joint3.attach(6);
    wrist_joint4.attach(7);       
    wrist_roll_joint5.attach(8);
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
  Serial.print(waist_val_raw);      
  Serial.print("\t");
  Serial.print(shoulder_val_raw);      
  Serial.print("\t");
  Serial.print(elbow_val_raw);      
  Serial.print("\t");
  Serial.print(wrist_val_raw); 
  Serial.print("\t");
  Serial.println(wrist_roll_val_raw);  

}
