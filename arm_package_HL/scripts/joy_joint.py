#!/usr/bin/env python3
import rospy
from math import pi
from arm_package.msg import Arm, Gun
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class Joystick(object):
    def __init__(self):
        self.g_joint_speed = pi / 2
        self.modes = True
        self.servo_gun_turn = 180
        self.servo_arm_turn = 1/15

        self.cam_distance = 0

        self.arm_pub = rospy.Publisher("/arm_data", Arm, queue_size=10)
        self.gun_pub = rospy.Publisher("/gun_data", Gun, queue_size=10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=10)
        self.cam_sub = rospy.Subscriber("/cam_distance", Int16, self.cam_cb, queue_size=10)

    def joy_cb(self, joy_msg):
        arm_data = Arm()
        gun_data = Gun()

        arm_data.Camera_Distance = self.cam_distance

        arm_mode         = joy_msg.buttons[10]
        gun_mode         = joy_msg.buttons[11]

        en_joints_button = joy_msg.buttons[1]
        gripper_button   = joy_msg.buttons[0]

        if (arm_mode):
            self.modes = True
        elif(gun_mode):
            self.modes = False

        if(self.modes): 
            arm_data.Arm_mode = True
            gun_data.Gun_mode = False

            arm_data.Joint_1     =  joy_msg.axes[6] * self.g_joint_speed           #waist joint
            arm_data.Joint_2     =  joy_msg.axes[1] * self.g_joint_speed           #shoulder joint
            arm_data.Joint_3     =  joy_msg.axes[3] * self.servo_arm_turn    #elbow joint
            arm_data.Joint_4     =  joy_msg.axes[7] * self.servo_arm_turn    #wrist joint
            arm_data.Joint_5     =  joy_msg.axes[0] * self.servo_arm_turn    #wrist_roll joint

            if (en_joints_button):
                arm_data.Motor_EN = True
            else:
                arm_data.Motor_EN = False

            if (gripper_button):
                arm_data.Gripper = True
            else:
                arm_data.Gripper = False

        else:
            arm_data.Arm_mode = False
            gun_data.Gun_mode = True
            gun_data.Turn = joy_msg.axes[0] * self.servo_gun_turn

            reload_button = joy_msg.buttons[3]
            shoot_button = joy_msg.buttons[0]

            if (reload_button):
                gun_data.Reload = True
            else:
                gun_data.Reload = False

            if(shoot_button):
                gun_data.Fire = True
            else:
                gun_data.Fire = False

        self.arm_pub.publish(arm_data)
        self.gun_pub.publish(gun_data)

    def cam_cb(self, cam_data):
        self.cam_distance = cam_data.data

if __name__ == "__main__":
    rospy.init_node("joint_joy", anonymous = False)
    rospy.loginfo("Starting manual control for the arm...")
    joy = Joystick()
    rospy.spin()