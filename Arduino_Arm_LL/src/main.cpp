#include <ros.h>                  //ROS API for cpp
#include <arm_package/Arm.h>
#include <arm_package/Gun.h>
#include <std_msgs/Float64.h>   
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <VarSpeedServo.h>
#include <AccelStepper.h>

#define elbow_pin      A2
#define wrist_pin      A3
#define wrist_roll_pin A4

#define waist       0 
#define shoulder    1
#define elbow       2
#define wrist       3
#define wrist_roll  4

#define safety 11
#define trigger 12

int16_t elbow_val_raw = 0,
        wrist_val_raw = 0,
        wrist_roll_val_raw = 0; 

float elbow_temp = 0,   //0
      wrist_temp = 0,   //0
      wrist_roll_temp = 0;  //0

AccelStepper    waist_joint1(1, 2, 3), //base
             shoulder_joint2(1, 4, 5); //shoulder

float joint[6] = {0, 0, 0, 0, 0};

VarSpeedServo elbow_joint3, wrist_joint4, wrist_roll_joint5, grip_servo,
              gun_servo;

int sensorValue = 0; 

bool  g_Joints_EN_Joy = false;  //global default for joint enable
const int Joints_EN_Pin = 13;   //out pin @ arduino mega

const int buttonOne = 12;

ros::NodeHandle  nh;                                          //object 
void arm_cb(const arm_package::Arm& Arm_data);   //init function
void gun_cb(const arm_package::Gun& Gun_data);
void read_analog(void);

ros::Subscriber<arm_package::Arm> Arm_sub("arm_data", &arm_cb);       //ros sub
ros::Subscriber<arm_package::Gun> Gun_sub("gun_data", &gun_cb); 

std_msgs::Float64 Test;
ros::Publisher test_pub("test", &Test);

void setup()
{
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
  //  velocity.data = drive_duty_cycle;
  //  vel_cmd.publish( &velocity );
  
  if (g_Joints_EN_Joy)                    
  {                                      
     digitalWrite(Joints_EN_Pin, LOW);
  }
  else
  {
    digitalWrite(Joints_EN_Pin, HIGH);
  }


  waist_joint1.runSpeed();
  shoulder_joint2.runSpeed();

  waist_joint1.setSpeed(joint[waist]);
  shoulder_joint2.setSpeed(joint[shoulder]);

  elbow_joint3.slowmove(joint[elbow], 15); //88 mid
  wrist_joint4.slowmove(joint[wrist], 15);
  wrist_roll_joint5.slowmove(joint[wrist_roll], 15);

  test_pub.publish(&Test);

  nh.spinOnce();
  //  delay(1);
}

void arm_cb(const arm_package::Arm& Arm_data)  //callback function from subscribe on driving each joins
{
  g_Joints_EN_Joy = Arm_data.Motor_EN;

  float base_joint       = Arm_data.Joint_1;       //-1.57 <-> 1.57
  float shoulder_joint   = Arm_data.Joint_2;  //-1.57 <-> 1.57
  float elbow_joint      = Arm_data.Joint_3;     //-1.57 <-> 1.57
  float wrist_joint      = Arm_data.Joint_4;     //-1.57 <-> 1.57
  float wrist_roll_joint = Arm_data.Joint_5;     //-1.57 <-> 1.57

  float base_degree = (base_joint * 180)/ PI;         // -90 <-> 90
  float shoulder_degree = (shoulder_joint * 180)/ PI;
  // float elbow_degree = (elbow_joint * 180)/ PI;
  // float wrist_degree = (wrist_joint * 180)/ PI;


  if(Arm_data.Gripper)
  {
    grip_servo.write(95);
  }
  else
  {
    grip_servo.write(40);
  }

  joint[waist] = map(base_degree, -90, 90, -1000, 1000);
  joint[shoulder] = map(shoulder_degree, -90, 90, -1000, 1000);
  // float elbow = map(elbow_degree, -90, 90, 36, 140);  //position
  // float wrist = map(wrist_degree, -90, 90, 34, 149);

  //=====================================================
  elbow_temp = elbow_temp + elbow_joint; //we have to send min 36 and max is 140
  wrist_temp += wrist_joint; //we have to send min 36 and max is 140
  wrist_roll_temp += wrist_roll_joint; //we have to send min 36 and max is 140

  joint[elbow] = elbow_temp + 88;
  joint[wrist] = wrist_temp + 88.5;
  joint[wrist_roll] = wrist_roll_temp + 90;

  if ( joint[elbow] <= 36) {      //elbow joint3
    joint[elbow] = 36;
  }
  else if (joint[elbow] >= 140){
    joint[elbow] = 140;
  }

  if ( joint[wrist] <= 34) {      //wrist joint4
    joint[wrist] = 34;
  }
  else if (joint[wrist] >= 143){
    joint[wrist] = 143;
  }

  if ( joint[wrist_roll] <= 35) {      //wrist_roll joint5
    joint[wrist_roll] = 35;
  }
  else if (joint[wrist_roll] >= 145){
    joint[wrist_roll] = 145;
  }
  
  Test.data =  joint[elbow];
  
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

void read_analog(void)
{
  
  elbow_val_raw = analogRead(elbow_pin) / (685 / 155);
  wrist_val_raw = analogRead(wrist_pin) / (685 / 155);  //685 max
  wrist_roll_val_raw = analogRead(wrist_roll_pin) / (685 / 155);
}