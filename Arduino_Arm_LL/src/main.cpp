#include <ros.h>                  //ROS API for cpp
#include <arm_package/Arm.h>
#include <arm_package/Gun.h>
#include <arm_package/Feedback.h>
#include <std_msgs/Float64.h>   
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <VarSpeedServo.h>
#include <AccelStepper.h>

#define shoulder_pin   A1


#define waist       0 
#define shoulder    1
#define elbow       2
#define wrist       3
#define wrist_roll  4

#define safety 11
#define trigger 12

AccelStepper    waist_joint1(1, 2, 3), //base
             shoulder_joint2(1, 4, 5); //shoulder

VarSpeedServo elbow_joint3, wrist_joint4, wrist_roll_joint5, grip_servo,
              gun_servo;

int16_t shoulder_val_raw =0,
        elbow_val_raw = 0,
        wrist_val_raw = 0,
        wrist_roll_val_raw = 0; 

int32_t elbow_temp = 0,   //0
      wrist_temp = 0,   //0
      wrist_roll_temp = 0;  //0

float joint[6] = {0, 0, 0, 0, 0};

int sensorValue = 0; 

bool  g_Joints_EN_Joy = false;  //global default for joint enable
const int Joints_EN_Pin = 13;   //out pin @ arduino mega

const int buttonOne = 12;

bool g_home = false;

ros::NodeHandle  nh;                                          //object 
void arm_cb(const arm_package::Arm& Arm_data);   //init function
void gun_cb(const arm_package::Gun& Gun_data);
void stepper_position_2(int target, int sensor, int velocity,int threshold);
int max_min(int val_raw, int lower, int higher)  ;

ros::Subscriber<arm_package::Arm> Arm_sub("arm_data", &arm_cb);       //ros sub
ros::Subscriber<arm_package::Gun> Gun_sub("gun_data", &gun_cb); 

std_msgs::Float64 Test;
ros::Publisher test_pub("test", &Test);

void setup()
{
  // nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(test_pub);
  nh.subscribe(Arm_sub);
  nh.subscribe(Gun_sub);

  // Serial.begin(9600);
  pinMode(Joints_EN_Pin, OUTPUT);
  waist_joint1.setMaxSpeed(1000000);
  shoulder_joint2.setMaxSpeed(1000000);
  elbow_joint3.attach(6);
  wrist_joint4.attach(7);
  wrist_roll_joint5.attach(8);
  grip_servo.attach(9);
  gun_servo.attach(10);

  joint[elbow]      = 88;
  joint[wrist]      = 88.5;
  joint[wrist_roll] = 90;

  pinMode(trigger, OUTPUT); //trigger
  pinMode(safety, OUTPUT); //safety
  pinMode(buttonOne, INPUT_PULLUP);

}

void loop()
{
  shoulder_val_raw   = analogRead(shoulder_pin) / (1023 / 500);
  
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
    joint[shoulder] = 1000;
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

  elbow_joint3.slowmove(joint[elbow], 15); //88 mid
  wrist_joint4.slowmove(joint[wrist], 15);
  wrist_roll_joint5.slowmove(joint[wrist_roll], 15);

  waist_joint1.runSpeed();
  shoulder_joint2.runSpeed();

  Test.data =  joint[elbow];
  test_pub.publish(&Test);

  nh.spinOnce();
}

void arm_cb(const arm_package::Arm& Arm_data)  //callback function from subscribe on driving each joins
{
  g_Joints_EN_Joy = Arm_data.Motor_EN;
  g_home          = Arm_data.Home;

  float base_joint       = Arm_data.Joint_1;       //-1.57 <-> 1.57
  float shoulder_joint   = Arm_data.Joint_2;     //-1.57 <-> 1.57
  float elbow_joint      = Arm_data.Joint_3;     //-1.57 <-> 1.57
  float wrist_joint      = Arm_data.Joint_4;     //-1.57 <-> 1.57
  float wrist_roll_joint = Arm_data.Joint_5;     //-1.57 <-> 1.57

  float base_degree = (base_joint * 180)/ PI;         // -90 <-> 90
  float shoulder_degree = (shoulder_joint * 180)/ PI;

  if(Arm_data.Gripper)
  {
    grip_servo.slowmove(95, 15);
  }
  else
  {
    grip_servo.slowmove(40, 15);
  }

  joint[waist]    = map(base_degree, -90, 90, -1000, 1000);
  joint[shoulder] = map(shoulder_degree, -90, 90, -1000, 1000);

  //=====================================================
  wrist_temp += wrist_joint; //we have to send min 36 and max is 140
  wrist_roll_temp += wrist_roll_joint; //we have to send min 36 and max is 140

  joint[elbow] = elbow_joint;
  joint[wrist] = wrist_temp + 88.5;
  joint[wrist_roll] = wrist_roll_temp + 90;

  joint[wrist]      = max_min(joint[wrist], 34, 143);
  joint[wrist_roll] = max_min(joint[wrist_roll], 35, 145);
}

void gun_cb(const arm_package::Gun& Gun_data)
{
  bool button1 = digitalRead(buttonOne);

  if(button1==false)
  {
    //bool button1 = digitalRead(buttonOne);
    digitalWrite(safety,HIGH); //activate safety
    //delay(500);
    digitalWrite(trigger, HIGH); //activate trigger
    delay(1);

    digitalWrite(safety, LOW); //disable safety
    digitalWrite(trigger, LOW);  //disable trigger
  }
}

void stepper_position_2(int sensor, int target, int velocity,int threshold)
{
  if (sensor > (target - threshold) && sensor < (target + threshold))
  {
    shoulder_joint2.setSpeed(0);
  }
  else
  {
    if( target < sensor)
     {
        shoulder_joint2.setSpeed(-1*velocity);
     }
     else
     {
        shoulder_joint2.setSpeed(velocity);
     }
  }
}

int max_min(int val_raw, int lower, int higher)  
{
  if ( val_raw <= lower) {
    val_raw = lower;
  }
  else if (val_raw >= higher){
    val_raw = higher;
  }
 return val_raw;
}

// uint16_t g_target_adc_raw[5] = {500, 500, 155, 155, 155};
// void read_analog(void)
// { 
//   //*********************************Stepper Analog***********************************
//   waist_val_raw      = analogRead(waist_pin)    / (1023 / g_target_adc_raw[0]);
//   shoulder_val_raw   = analogRead(shoulder_pin) / (1023 / g_target_adc_raw[1]);
//   //*********************************Stepper Analog***********************************
  
//   //*********************************Servo Analog***********************************
//   elbow_val_raw      = analogRead(elbow_pin)      / (685 / g_target_adc_raw[2]);
//   wrist_val_raw      = analogRead(wrist_pin)      / (685 / g_target_adc_raw[3]);  //685 max
//   wrist_roll_val_raw = analogRead(wrist_roll_pin) / (685 / g_target_adc_raw[4]);
// }
