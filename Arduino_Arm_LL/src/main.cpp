#include <ros.h>                  //ROS API for cpp
#include <arm_package/Arm.h>
#include <arm_package/Gun.h>
#include <SoftwareSerial.h>
#include <VarSpeedServo.h>
#include <AccelStepper.h>

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
int max_min(int val_raw, int lower, int higher);

ros::Subscriber<arm_package::Arm> Arm_sub("arm_data", &arm_cb);       //ros sub
ros::Subscriber<arm_package::Gun> Gun_sub("gun_data", &gun_cb); 

// arm_package::Feedback Potentiometer;
// ros::Publisher Pot_pub("pot", &Potentiometer);
void setup()
{
  nh.initNode();
  nh.subscribe(Arm_sub);
  nh.subscribe(Gun_sub);
  // nh.advertise(Pot_pub);

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

  // Pot_pub.publish(&Potentiometer);

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
    grip_servo.slowmove(95, 25);
  }
  else
  {
    grip_servo.slowmove(40, 25);
  }
  
  //=====================================================Velocity
    int16_t waist_vel    = map(waist_joint,    120, 173, -1000, 1000);
    int16_t shoulder_vel = map(shoulder_joint, 29,   67, -1000, 1000);

    joint[waist]      = waist_vel;
    joint[shoulder]   = shoulder_vel;
    joint[elbow]      = elbow_joint;
    joint[wrist]      = wrist_joint;
    joint[wrist_roll] = wrist_roll_joint;
}

int16_t pos = 0;
void gun_cb(const arm_package::Gun& Gun_data)
{
  // bool button1 = digitalRead(fire);

  gun_swivel_servo.slowmove(Gun_data.Turn, 20);

  if(Gun_data.Fire)
  {
    digitalWrite(11,HIGH); //activate safety
    // delay(100);
    digitalWrite(12, HIGH); //activate trigger
  }
  else
  {
    digitalWrite(11, LOW); //disable safety
    digitalWrite(12, LOW);  //disable trigger
  }


  if (Gun_data.Reload)    //true
  {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      reload_servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      reload_servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

}

// void stepper_position_1(int sensor, int target, int velocity,int threshold)
// {
//   if (sensor > (target - threshold) && sensor < (target + threshold))
//   {
//     waist_joint1.setSpeed(0);
//   }
//   else
//   {
//      if( target > sensor)
//      {
//         waist_joint1.setSpeed(-1*velocity);
//      }
//      else
//      {
//         waist_joint1.setSpeed(velocity);
//      }
//   }
// }

// void stepper_position_2(int sensor, int target, int velocity,int threshold)
// {
//   if (sensor > (target - threshold) && sensor < (target + threshold))
//   {
//     shoulder_joint2.setSpeed(0);
//   }
//   else
//   {
//     if( target < sensor)
//      {
//         shoulder_joint2.setSpeed(-1*velocity);
//      }
//      else
//      {
//         shoulder_joint2.setSpeed(velocity);
//      }
//   }
// }

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
