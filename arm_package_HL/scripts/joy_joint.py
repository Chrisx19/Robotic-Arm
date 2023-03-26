#!/usr/bin/env python3
import rospy
from arm_package.msg import Joints
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Joystick(object):
    def __init__(self):
        self.g_joint_speed = 1000
        # self.g_joint_en = False

        self.cmd_pub = rospy.Publisher("/joints", Joints, queue_size=10)
        # self.EN_pub  = rospy.Publisher("/joints_EN", Bool, queue_size = 10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=10)

    def joy_cb(self, joy_msg):
        joint_drive = Joints()

        joint1           =  joy_msg.axes[1]*self.g_joint_speed          #base joint
        joint2           =  joy_msg.axes[4]*self.g_joint_speed*-1       #shoulder joint
        en_pub_button    = joy_msg.buttons[5]
        en_joints_button = joy_msg.buttons[0]

        joint_drive.Joint_1 = int(joint1)
        joint_drive.Joint_2 = int(joint2)

        if(en_pub_button > 0): 
            
            if (en_joints_button > 0):
                joint_drive.EN = True
                self.cmd_pub.publish(joint_drive)
            else:
                joint_drive.EN = False
                self.cmd_pub.publish(joint_drive)

if __name__ == "__main__":
    rospy.init_node("joint_joy", anonymous = False)
    rospy.loginfo("Starting manual control for the arm...")
    joy = Joystick()
    rospy.spin()
