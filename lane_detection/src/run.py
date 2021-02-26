#!/usr/bin/env python
import rospy
import os
import traceback
import time
import math

from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class DriverNode():

    def __init__(self, type="path"):

        self.type = "path"

        self.min_vel_x = -0.2
        self.max_vel_x = 2.0
        self.min_theta = -20
        self.max_theta = 20
        self.ranges = []

        self.mode = "MANUAL"

        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=10)

        rospy.Subscriber("/lane_detection/path", Path, self.pose_callback, queue_size=10)

        self.ack_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

        self.wheelbase = 0.325




    def joy_callback(self, joy):
    
        if(joy.buttons[5]):
            self.mode = "AUTO"
        
        elif(joy.buttons[4]):
            self.mode = "MANUAL"

        else:
            self.mode = "DISABLED"


    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0

        radius = v / omega
        return math.atan(wheelbase / radius)
        

    def pub_command(self, vel_x, vel_y, orientation):

        ack_msg = AckermannDriveStamped()

        ack_msg.header.frame_id = "odom"

        steer = self.convert_trans_rot_vel_to_steering_angle(vel_x, vel_y, self.wheelbase)

        print("\nCalculated Velocity : {}".format(vel_x))
        print("Calculated Steering Angle : {}".format(math.degrees(steer)))

        cmd_vel_x = max(min(self.max_vel_x, vel_x), self.min_vel_x)
        steering_angle = max(min(math.radians(self.max_theta), steer), math.radians(self.min_theta))

        self.vel_x = 1.0/2
        
        self.steering = 1*(steering_angle + orientation)

        ack_msg.drive.speed = self.vel_x


        ack_msg.drive.steering_angle = self.steering

        print(self.mode)
        print("\nVelocity : {}".format(cmd_vel_x))
        print("Steering Angle : {}".format(math.degrees(steering_angle)))


        if(self.mode == "AUTO"):
            self.ack_pub.publish(ack_msg)
        else:
            pass



    def pose_callback(self, msg):

        if self.type == "path":

            target_x = msg.poses[0].pose.position.x 
            target_y = msg.poses[0].pose.position.y 

            ori = msg.poses[0].pose.orientation

            target_ori = math.degrees(euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2])

            target_ori = 0      ####################

            vel_x = target_x/20 

            vel_y = target_y/20

            vel_x = vel_x * 0.4
            vel_y = vel_y * -0.15

            print(vel_y)

            self.pub_command(vel_x, vel_y, target_ori)





if __name__ == "__main__":

    rospy.init_node("run_node")

    sender = DriverNode()

    
    rospy.spin()

