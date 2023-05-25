#include <ros.h>                  //ROS API for cpp
#include <arm_package/Arm.h>
#include <arm_package/Gun.h>
#include <arm_package/Feedback.h>>
#include <SoftwareSerial.h>
#include <VarSpeedServo.h>
#include <AccelStepper.h>

#define waist_pin      A0
#define shoulder_pin   A1
#define elbow_pin      A2
#define wrist_pin      A3
#define wrist_roll_pin A4

#define waist       0 
#define shoulder    1
#define elbow       2
#define wrist       3
#define wrist_roll  4

AccelStepper    waist_joint1(1, 2, 3), //base
             shoulder_joint2(1, 4, 5); //shoulder

VarSpeedServo elbow_joint3, wrist_joint4, wrist_roll_joint5, grip_servo,
              gun_swivel_servo, reload_servo;

int16_t waist_val_raw      = 0,
        shoulder_val_raw   = 0,
        elbow_val_raw      = 0,
        wrist_val_raw      = 0,
        wrist_roll_val_raw = 0;  

int32_t elbow_temp = 0,   //0
      wrist_temp = 0,   //0
      wrist_roll_temp = 0;  //0

float joint[6]    = {0, 0, 0, 0, 0};

const int Joints_EN_Pin = 13;   //out pin @ arduino mega

bool g_Joints_EN_Joy = false;  //global default for joint enable
bool g_home = false;
bool g_auto_mode = false;

ros::NodeHandle  nh;                                          //object 
void arm_cb(const arm_package::Arm& Arm_data);   //init function
void gun_cb(const arm_package::Gun& Gun_data);
int16_t max_min(int val_raw, int lower, int higher);
void read_analog(void);

ros::Subscriber<arm_package::Arm> Arm_sub("arm_data", &arm_cb);       //ros sub
ros::Subscriber<arm_package::Gun> Gun_sub("gun_data", &gun_cb); 

arm_package::Feedback Potentiometer;
ros::Publisher Pot_pub("pot", &Potentiometer);
void setup()
{
  nh.initNode();
  nh.subscribe(Arm_sub);
  nh.subscribe(Gun_sub);
  nh.advertise(Pot_pub);

  pinMode(Joints_EN_Pin, OUTPUT);
  waist_joint1.setMaxSpeed(1000000);
  shoulder_joint2.setMaxSpeed(1000000);
  elbow_joint3.attach(6);
  wrist_joint4.attach(7);       //servo
  wrist_roll_joint5.attach(8);  //servo
  grip_servo.attach(9);         //servo

  gun_swivel_servo.attach(10);         //servo
  reload_servo.attach(23);      //reload_servo

  joint[elbow]      = 88;
  joint[wrist]      = 88.5;
  joint[wrist_roll] = 90;

//*****************GUN SETUP************************
  pinMode(11, OUTPUT);      //safety
  pinMode(12, OUTPUT);     //trigger
//*****************GUN SETUP************************
}

void loop()
{
  read_analog();
  if (g_Joints_EN_Joy)                    
  {                                      
     digitalWrite(Joints_EN_Pin, LOW);
  }
  else
  {
    digitalWrite(Joints_EN_Pin, HIGH);
  }

  if (g_home)
  {
    joint[shoulder] = 500;
    joint[elbow]      = 88;
    joint[wrist]      = 88.5;
    joint[wrist_roll] = 90;
  }
  else
  {
    joint[elbow]      = joint[elbow];
    joint[wrist]      = joint[wrist];
    joint[wrist_roll] = joint[wrist_roll];
  }

  waist_joint1.setSpeed(joint[waist]);
  shoulder_joint2.setSpeed(joint[shoulder]);

  elbow_joint3.slowmove(joint[elbow], 25); //88 mid
  wrist_joint4.slowmove(joint[wrist], 15);
  wrist_roll_joint5.slowmove(joint[wrist_roll], 15);

  waist_joint1.runSpeed();
  shoulder_joint2.runSpeed();

  Pot_pub.publish(&Potentiometer);

  nh.spinOnce();
}

void arm_cb(const arm_package::Arm& Arm_data)  //callback function from subscribe on driving each joins
{
  g_Joints_EN_Joy = Arm_data.Motor_EN;
  g_home          = Arm_data.Home;

  float waist_joint       = Arm_data.Joint_1;       //-1.57 <-> 1.57
  float shoulder_joint   = Arm_data.Joint_2;     //-1.57 <-> 1.57
  float elbow_joint      = Arm_data.Joint_3;     //-1.57 <-> 1.57
  float wrist_joint      = Arm_data.Joint_4;     //-1.57 <-> 1.57
  float wrist_roll_joint = Arm_data.Joint_5;     //-1.57 <-> 1.57

  if(Arm_data.Gripper)
  {
    grip_servo.slowmove(105, 25);
  }
  else
  {
    grip_servo.slowmove(40, 25);
  }
  
  //=====================================================Velocity
    joint[waist]      = waist_joint;
    joint[shoulder]   = shoulder_joint;
    joint[elbow]      = elbow_joint;
    joint[wrist]      = wrist_joint;
    joint[wrist_roll] = wrist_roll_joint;
}

int16_t pos = 0;
void gun_cb(const arm_package::Gun& Gun_data)
{
  gun_swivel_servo.slowmove(Gun_data.Turn, 20);

  if(Gun_data.Safety)
  {
    digitalWrite(11,HIGH); //activate safety
  }
  else
  {
    digitalWrite(11, LOW); //disable safety
  }

  if(Gun_data.Fire)
  {
    digitalWrite(12, HIGH); //activate trigger
  }
  else
  {
    digitalWrite(12, LOW);  //disable trigger
  }


  if (Gun_data.Reload)    //true
  {
    reload_servo.slowmove(0, 30);              // tell servo to go to position in variable 'pos'
  }
  else
  {
    reload_servo.slowmove(180, 30);
  }

}

int16_t max_min(int val_raw, int lower, int higher)  
{
  if ( val_raw <= lower) {
    val_raw = lower;
  }
  else if (val_raw >= higher){
    val_raw = higher;
  }
 return val_raw;
}

uint16_t g_target_adc_raw[5] = {250, 250, 155, 155, 155};
void read_analog(void)
{ 
  //*********************************Stepper Analog***********************************
  waist_val_raw      = analogRead(waist_pin)    / (1023 / g_target_adc_raw[0]);
  shoulder_val_raw   = analogRead(shoulder_pin) / (1023 / g_target_adc_raw[1]);
  //*********************************Stepper Analog***********************************
  
  //*********************************Servo Analog***********************************
  elbow_val_raw      = analogRead(elbow_pin)      / (685 / g_target_adc_raw[2]);
  wrist_val_raw      = analogRead(wrist_pin)      / (685 / g_target_adc_raw[3]);  //685 max
  wrist_roll_val_raw = analogRead(wrist_roll_pin) / (685 / g_target_adc_raw[4]);

  Potentiometer.Pot_1 = waist_val_raw;
  Potentiometer.Pot_2 = shoulder_val_raw;
  Potentiometer.Pot_3 = elbow_val_raw;
  Potentiometer.Pot_4 = wrist_val_raw;
  Potentiometer.Pot_5 = wrist_roll_val_raw;
}
