#include <ros.h>                  //ROS API for cpp
#include <arm_package/Joints.h>
#include <std_msgs/Int32.h>   
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>

#include <AccelStepper.h>

AccelStepper stepper1(1, 2, 3);
AccelStepper stepper2(1, 4, 5);

bool  g_Joints_EN_Joy = false;  //global default for joint enable
const int Joints_EN_Pin = 13;   //out pin @ arduino mega

ros::NodeHandle  nh;                                          //object 
void arm_joint_cb(const arm_package::Joints& Joints_data);   //init function

ros::Subscriber<arm_package::Joints> Arm_joint_sub("joints", &arm_joint_cb);       //ros sub

//std_msgs::Int32 velocity;
//ros::Publisher vel_cmd("vel_cmd", &velocity);

void setup()
{
  nh.initNode();
  //  nh.advertise(vel_cmd);
  nh.subscribe(Arm_joint_sub);

  pinMode(Joints_EN_Pin, OUTPUT);
  stepper1.setMaxSpeed(1000000);
  stepper2.setMaxSpeed(1000000);
  
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
  int forward_j_1 = Joints_data.Joint_1;  //-1000 <-> 1000
  int forward_j_2 = Joints_data.Joint_2;  //-1000 <-> 1000

  if (Joints_data.EN)
  {
    g_Joints_EN_Joy = true;
  }
  else
  {
    g_Joints_EN_Joy = false;
  }

  stepper1.setSpeed(forward_j_1);
  stepper2.setSpeed(forward_j_2);
}