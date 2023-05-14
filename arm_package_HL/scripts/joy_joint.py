#!/usr/bin/env python3
import rospy
from math import pi
from arm_package.msg import HL_Data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Joystick(object):
    def __init__(self):
        self.g_joint_speed = pi / 2
        self.mode = True
        # self.g_joint_en = False

        self.hl_pub = rospy.Publisher("/arm", HL_Data, queue_size=10)
        # self.EN_pub  = rospy.Publisher("/joints_EN", Bool, queue_size = 10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=10)

    def joy_cb(self, joy_msg):
        hl_data = HL_Data()

        joint1           =  joy_msg.axes[6] * self.g_joint_speed
        joint2           =  joy_msg.axes[1] * self.g_joint_speed           #shoulder joint
        joint3           =  joy_msg.axes[3] * self.g_joint_speed  * 1/3    #elbow joint
        joint4           =  joy_msg.axes[7] * self.g_joint_speed  * 1/3    #elbow joint
        joint5           =  joy_msg.axes[0] * self.g_joint_speed  * 1/3    #elbow joint

        arm_mode         = joy_msg.buttons[10]
        gun_mode         = joy_msg.buttons[11]

        en_pub_button    = joy_msg.buttons[6]
        en_joints_button = joy_msg.buttons[1]
        gripper_button   = joy_msg.buttons[0]

        hl_data.Joint_1 = joint1
        hl_data.Joint_2 = joint2
        hl_data.Joint_3 = joint3
        hl_data.Joint_4 = joint4
        hl_data.Joint_5 = joint5

        if (arm_mode > 0):
            self.mode = True
        elif(gun_mode > 0):
            self.mode = False

        if(self.mode): 
            if (en_joints_button > 0):
                hl_data.EN = True
                # self.cmd_pub.publish(hl_data)
            else:
                hl_data.EN = False
                

            if (gripper_button > 0):
                hl_data.Gripper = True
            else:
                hl_data.Gripper = False

            self.hl_pub.publish(hl_data)
        else:
            print('gun mode')


if __name__ == "__main__":
    rospy.init_node("joint_joy", anonymous = False)
    rospy.loginfo("Starting manual control for the arm...")
    joy = Joystick()
    rospy.spin()