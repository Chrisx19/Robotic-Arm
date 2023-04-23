#include <ros.h>                  //ROS API for cpp
#include <arm_package/Joints.h>
#include <std_msgs/Float64.h>   
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>
#include <math.h>

#include <Servo.h>
#include <AccelStepper.h>

AccelStepper stepper1(1, 2, 3); //base
AccelStepper stepper2(1, 4, 5); //shoulder
Servo elbow_servo, wrist_servo;                  //Elbow

bool  g_Joints_EN_Joy = false;  //global default for joint enable
const int Joints_EN_Pin = 13;   //out pin @ arduino mega

ros::NodeHandle  nh;                                          //object 
void arm_joint_cb(const arm_package::Joints& Joints_data);   //init function

ros::Subscriber<arm_package::Joints> Arm_joint_sub("joints", &arm_joint_cb);       //ros sub

std_msgs::Float64 Test;
ros::Publisher test_pub("test", &Test);

void setup()
{
  nh.initNode();
   nh.advertise(test_pub);
  nh.subscribe(Arm_joint_sub);

  // Serial.begin(9600);
  pinMode(Joints_EN_Pin, OUTPUT);
  stepper1.setMaxSpeed(1000000);
  stepper2.setMaxSpeed(1000000);
  elbow_servo.attach(9);
  // elbow_servo.write(34);  //90~ 0degree is at home, 149~90 degree  34~-90 degrees
  
}

void loop()
{
  //  velocity.data = drive_duty_cycle;
  //  vel_cmd.publish( &velocity );
  
/*The condition below is when we pressed 'A' on xbox, then the motor will be disabled.
That means we can manually move the motor with our hands and read the encoder val for better 
                              autonomus assistance */
  if (g_Joints_EN_Joy)                    
  {                                      
     digitalWrite(Joints_EN_Pin, HIGH);
  }
  else
  {
    digitalWrite(Joints_EN_Pin, LOW);
  }

  stepper1.runSpeed();
  stepper2.runSpeed();
  nh.spinOnce();
  //  delay(1);
}

void arm_joint_cb(const arm_package::Joints& Joints_data)  //callback function from subscribe on driving each joins
{
  float base_joint = Joints_data.Joint_1;       //-1.57 <-> 1.57
  float shoulder_joint = Joints_data.Joint_2;  //-1.57 <-> 1.57
  float elbow_joint = Joints_data.Joint_3;     //-1.57 <-> 1.57
  // float wrist_joint = Joints_data.Joint_4;     //-1.57 <-> 1.57

  float base_degree = (base_joint * 180)/ PI;
  float shoulder_degree = (shoulder_joint * 180)/ PI;
  float elbow_degree = (elbow_joint * 180)/ PI;
  // float wrist_degree = (wrist_joint * 180)/ PI;

  float base = map(base_degree, -90, 90, -1000, 1000);
  float shoulder = map(shoulder_degree, -90, 90, -1000, 1000);
  float elbow = map(elbow_degree, -90, 90, 34, 149);
  // float wrist = map(wrist_degree, -90, 90, 34, 149);

  Test.data = base;
  test_pub.publish(&Test);
  if (Joints_data.EN)
  {
    g_Joints_EN_Joy = true;
  }
  else
  {
    g_Joints_EN_Joy = false;
  }

  stepper1.setSpeed(base);
  stepper2.setSpeed(shoulder);
  elbow_servo.write(elbow);
  // wrist_servo.write(wrist);
}