#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Joystick(object):
    def __init__(self):
        self.g_joint_speed = 1000
        self.g_joint_en = False

        self.cmd_pub = rospy.Publisher("/joints", Twist, queue_size=10)
        self.EN_pub  = rospy.Publisher("/joints_EN", Bool, queue_size = 10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=10)

    def joy_cb(self, joy_msg):
        joint_drive = Twist()

        joint1    =  joy_msg.axes[1]*self.g_joint_speed
        en_pub_button = joy_msg.buttons[5]
        en_joints_button = joy_msg.buttons[0]

        joint_drive.linear.x = int(joint1)

        if(en_pub_button > 0): 
            self.cmd_pub.publish(joint_drive)
            if (en_joints_button > 0):
                self.g_joint_en = True
                self.EN_pub.publish(self.g_joint_en)
            else:
                self.g_joint_en = False
                self.EN_pub.publish(self.g_joint_en)

if __name__ == "__main__":
    rospy.init_node("joint_joy", anonymous=False)
    rospy.loginfo("Starting manual control for the arm...")
    joy = Joystick()
    rospy.spin()