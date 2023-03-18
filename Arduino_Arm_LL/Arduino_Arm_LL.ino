#include <ros.h>                  //ROS API for cpp
#include <geometry_msgs/Twist.h>  //The one moving the motors from joystick
#include <std_msgs/Int32.h>   
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>

#include <AccelStepper.h>

AccelStepper stepper;

bool  g_Joints_EN_Joy = false;  //global default for joint enable
const int Joints_EN_Pin = 13;   //out pin @ arduino mega

ros::NodeHandle  nh;                                          //object 
void arm_joint_cb(const geometry_msgs::Twist& Joints_data);   //init function
void joints_EN_cb(const std_msgs::Bool&       Joints_EN);     //init function

ros::Subscriber<std_msgs::Bool>       joint_EN_sub("joints_EN", &joints_EN_cb);     //ros sub
ros::Subscriber<geometry_msgs::Twist> Arm_joint_sub("joints", &arm_joint_cb);       //ros sub

//std_msgs::Int32 velocity;
//ros::Publisher vel_cmd("vel_cmd", &velocity);

void setup()
{
  nh.initNode();
  //  nh.advertise(vel_cmd);
  nh.subscribe(joint_EN_sub);
  nh.subscribe(Arm_joint_sub);

  pinMode(Joints_EN_Pin, OUTPUT);
  stepper.setMaxSpeed(1000000);
  
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

  stepper.runSpeed();
  nh.spinOnce();
  //  delay(1);
}

void arm_joint_cb(const geometry_msgs::Twist& Joints_data)  //callback function from subscribe on driving each joins
{
  int forward_x = Joints_data.linear.x;

  stepper.setSpeed(forward_x);
}

void joints_EN_cb(const std_msgs::Bool& Joints_EN)        //callback from joystick message from high level control
{
  if (Joints_EN.data)
  {
    g_Joints_EN_Joy = true;
  }
  else
  {
    g_Joints_EN_Joy = false;
  }
}
